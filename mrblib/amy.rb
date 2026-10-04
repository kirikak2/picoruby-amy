# AMY synthesizer
#
# The AMY engine (https://github.com/shorepine/amy) running on its own
# audio task and playing out of the board's I2S output.
#
# Three ways in, from low to high level:
#
#   AMY.command(synth: 1, patch: 130, num_voices: 6)   # any AMY parameter
#   AMY.wire("i1K130iv6Z")                              # raw wire message
#
#   fm = AMY::FM.new(channel: 0, voices: 6, patch: 128) # FM synth object
#   fm.algorithm = 5
#   fm.op(2).attack = 30
#   fm.map_cc(74, :filter_freq)                         # CC -> parameter, inside AMY
#
# and as a MIDI transport, so it plugs into picoruby-midi like any other
# MIDI device:
#
#   dev = MIDI::Device.new(AMY::Transport.instance)   # or MIDI::Device.new(fm.transport)
#   dev.note_on(60, 100)
#   dev.trigger(64, 100, duration: 200)
#
# MIDI channel N (0-based, as in MIDI::Device) plays AMY synth N + 1, and
# only once that synth has been set up (AMY::FM.new or AMY.command(synth:)).
module AMY
  # Wire codes for AMY.command. A subset of AMY's docs/api.md tables:
  # synths, oscillators, FM, filters, envelopes and the global effects.
  WIRE = {
    synth: "i", num_voices: "iv", oscs_per_voice: "in", synth_level: "iV",
    midi_cc: "ic", pedal: "ip", patch: "K", synth_flags: "if",
    osc: "v", wave: "w", note: "n", vel: "l", freq: "f", amp: "a",
    bp0: "A", bp1: "B", eg0_type: "T", eg1_type: "X",
    feedback: "b", algorithm: "o", algo_source: "O", ratio: "I",
    filter_type: "G", filter_freq: "F", resonance: "R",
    pan: "Q", portamento: "m", duty: "d", phase: "P", preset: "p",
    mod_source: "L", chained_osc: "c", bus: "y",
    volume: "V", reverb: "h", chorus: "k", echo: "M", eq: "x",
    reset: "S"
  }

  # AMY's PARAM_* numbers (amy/constants.py) for the direct form of midi_cc.
  PARAM_AMP          = 3
  PARAM_DUTY         = 13
  PARAM_FEEDBACK     = 23
  PARAM_FREQ         = 24
  PARAM_PAN          = 38
  PARAM_FILTER_FREQ  = 48
  PARAM_RATIO        = 58
  PARAM_RESONANCE    = 59
  PARAM_PORTAMENTO   = 60
  PARAM_VOLUME       = 71
  PARAM_ECHO_LEVEL   = 210
  PARAM_CHORUS_LEVEL = 215
  PARAM_REVERB_LEVEL = 219

  # synth_flags bit: no "note off does not match note on" style warnings on
  # stderr. Those print on whichever task delivered the note -- for
  # trigger()'s automatic note-off that is the esp_timer task on core 0, and
  # with the console on USB CDC a write from that core breaks the console.
  SYNTH_FLAGS_NO_NOTE_WARNINGS = 8

  # --- Engine -------------------------------------------------------------

  # Start the engine (idempotent). Returns true when it is running.
  def self.start
    return true if _running
    _start
  end

  def self.running?
    _running
  end

  # Back to the power-on state: every synth, patch, CC mapping and effect is
  # cleared. Audio keeps running.
  def self.reset
    _reset
    # The reset is queued and runs on the audio task's next block, while a
    # patch load (AMY::FM.new, fm.patch =) runs right away on this task. Wait
    # for the reset to land, or it wipes a synth set up next.
    wait_blocks(2)
    nil
  end

  # Wait until the audio task has rendered `n` more blocks (at most 100 ms),
  # i.e. until everything queued so far has been applied.
  def self.wait_blocks(n)
    start = _blocks
    waited = 0
    while _blocks - start < n && waited < 100
      sleep_ms 2
      waited += 2
    end
  end

  # Smoothed fraction of real time spent rendering (1.0 = no headroom).
  def self.render_load
    _render_load
  end

  # How many times the overload failsafe has reset the engine.
  def self.overloads
    _overloads
  end

  # Blocks rendered since start (one per 5.8 ms). If this stops climbing,
  # the audio task is stuck.
  def self.blocks
    _blocks
  end

  # A short two-tone bleep, to check that sound comes out.
  def self.bleep
    _bleep
  end

  # --- Messages -----------------------------------------------------------

  # Send a raw AMY wire message. A missing trailing "Z" is added.
  def self.wire(message)
    message = message + "Z" unless message[message.length - 1] == "Z"
    _send(message) == 0
  end

  # Send one event built from AMY parameter names (see WIRE), e.g.
  #   AMY.command(synth: 1, osc: 0, filter_freq: 2000)
  # Arrays become comma lists, nil elements leave that slot unchanged.
  def self.command(**params)
    command_hash(params)
  end

  # As command, with the parameters in a Hash (keys may come from variables).
  def self.command_hash(params)
    msg = ""
    # The synth has to come first: everything after it is addressed to it.
    synth = params[:synth]
    msg << "i" << format_value(synth) unless synth.nil?
    params.each do |key, value|
      next if key == :synth || value.nil?
      code = WIRE[key]
      raise ArgumentError, "AMY: unknown parameter #{key}" if code.nil?
      msg << code << format_value(value)
    end
    wire(msg)
  end

  def self.format_value(value)
    if value.is_a?(Array)
      value.map { |v| v.nil? ? "" : format_value(v) }.join(",")
    elsif value == true
      "1"
    elsif value == false
      "0"
    elsif value.is_a?(Float)
      # mruby/c prints floats with "%g", which turns to exponent notation
      # below 1e-4 and from 1e6 up -- and in a wire message the "e" would
      # be read as the next parameter letter. Keep every float out of that
      # range: too small to matter -> 0, very large -> integer.
      if value > -0.0001 && value < 0.0001
        "0"
      elsif value >= 100000.0 || value <= -100000.0
        value.to_i.to_s
      else
        value.to_s
      end
    else
      value.to_s
    end
  end

  # --- Helpers ------------------------------------------------------------

  # The inverse of scale: where `value` sits on min..max, as 0.0..127.0 --
  # for putting a parameter read back from AMY on a knob.
  #   UI.knob_set(5, AMY.unscale(fm.op(1).attack, 1, 2000, log: true), notify: false)
  def self.unscale(value, min, max, log: false)
    t = if log
          _log_unscale(value, min, max)
        else
          (value - min).to_f / (max - min)
        end
    t = 0.0 if t < 0.0
    t = 1.0 if t > 1.0
    t * 127.0
  end

  # Map a 0..127 controller value onto min..max (log: true for frequencies
  # and times, where equal steps should sound equal).
  #   AMY.scale(v, 1, 2000, log: true)
  def self.scale(value, min, max, log: false)
    t = value.to_f / 127.0
    t = 0.0 if t < 0.0
    t = 1.0 if t > 1.0
    if log
      _log_scale(t, min, max)
    else
      min + (max - min) * t
    end
  end

  # ==========================================================================
  # AMY::Transport -- the engine as a picoruby-midi transport
  # ==========================================================================
  class Transport
    def self.instance
      $__amy_transport_instance__ = new if $__amy_transport_instance__.nil?
      $__amy_transport_instance__
    end

    def initialize
      raise "AMY: failed to start the audio engine" unless AMY.start
    end

    # This transport's bit in picoruby-midi's transport registry; what
    # MIDI::Device#trigger and the note scheduler route by.
    def transport_id
      AMY._transport_bit
    end

    def send_packet(cable, cin, midi1, midi2, midi3)
      AMY._send_packet(cable, cin, midi1, midi2, midi3)
    end

    # Send-only: AMY produces no MIDI input.
    def bytes_available
      0
    end

    def read_available
      nil
    end

    def connected?
      AMY._running
    end

    def device_info
      "AMY synthesizer"
    end
  end

  # ==========================================================================
  # AMY::FM -- a DX7-style FM synth on one MIDI channel
  # ==========================================================================
  #
  # Built on AMY's DX7 patches (128..255). Their voice layout: osc 0 is the
  # ALGO osc (algorithm, feedback, and here also the filter), osc 1 the LFO,
  # oscs 2..7 the six operators -- osc 2 is DX7 operator 6 and osc 7 is
  # operator 1, because AMY's algo_source lists operators from 6 down.
  # op(n) uses DX7 numbering (1..6) and does that translation.
  #
  # Parameters set here are commands to the whole synth, so they reach every
  # voice at once.
  class FM
    ALGO_OSC = 0

    FILTER_TYPES = {
      none: 0, lowpass: 1, bandpass: 2, highpass: 3, lowpass24: 4, notch: 5
    }

    # param => [AMY PARAM number, target (:algo / :op / :bus), min, max, log]
    CC_TARGETS = {
      filter_freq: [PARAM_FILTER_FREQ,  :algo, 100,  8000, true],
      resonance:   [PARAM_RESONANCE,    :algo, 0.7,  8.0,  false],
      feedback:    [PARAM_FEEDBACK,     :algo, 0.0,  1.0,  false],
      pan:         [PARAM_PAN,          :algo, 0.0,  1.0,  false],
      portamento:  [PARAM_PORTAMENTO,   :algo, 0,    500,  false],
      level:       [PARAM_AMP,          :op,   0.0,  1.0,  false],
      ratio:       [PARAM_RATIO,        :op,   0.5,  16.0, true],
      volume:      [PARAM_VOLUME,       :bus,  0.0,  1.0,  false],
      reverb:      [PARAM_REVERB_LEVEL, :bus,  0.0,  1.0,  false],
      chorus:      [PARAM_CHORUS_LEVEL, :bus,  0.0,  1.0,  false]
    }

    attr_reader :channel, :voices, :patch, :transport

    # channel: MIDI channel, 0-based (AMY synth channel + 1)
    # voices:  polyphony
    # patch:   AMY patch number, DX7 presets are 128..255
    def initialize(channel: 0, voices: 6, patch: 128)
      @transport = AMY::Transport.instance
      @channel = channel
      @number = channel + 1
      @voices = voices
      @ops = {}
      self.patch = patch
    end

    # The AMY synth number this object drives (channel + 1).
    def synth_number
      @number
    end

    # Load a patch. Everything set through this object since the last load
    # is replaced by the patch's own values, read back from AMY (see
    # refresh), so algorithm, feedback and op(n).level / ratio / attack ...
    # report the preset. Takes a couple of audio blocks (~12 ms).
    def patch=(number)
      AMY.command(synth: @number, num_voices: @voices, patch: number)
      # After the load: a patch may carry flags of its own.
      AMY.command(synth: @number, synth_flags: SYNTH_FLAGS_NO_NOTE_WARNINGS)
      @patch = number
      @values = {}
      @filter = :none
      @ops.each { |_n, op| op._forget }
      refresh
    end

    # Read the synth's FM state back from AMY: algorithm, feedback, and each
    # operator's level, ratio and envelope. A patch's per-osc settings land
    # on the audio task's next block, so this waits for two blocks first.
    #
    # A DX7 envelope has more stages than an ADSR; it is summarised as
    # attack = the first stage's time, decay = the middle stages' times,
    # sustain = the level held until note-off, release = the last stage's
    # time. The operator then keeps those, so changing one stage starts the
    # others from the preset rather than from the defaults.
    # @return [Boolean] false if AMY has no such synth
    def refresh
      AMY.wait_blocks(2)
      text = AMY._fm_state(@number)
      return false if text.nil?
      text.split("\n").each do |line|
        f = line.split(" ")
        case f[0]
        when "algo"
          @values[:algorithm] = f[1].to_i
        when "fb"
          @values[:feedback] = f[1].to_f
        when "op"
          n = 8 - f[1].to_i   # osc 7 is DX7 operator 1
          op(n)._load(f[2].to_f, f[3].to_f, f[4]) if n >= 1 && n <= 6
        end
      end
      true
    end

    # Operator n (DX7 numbering, 1..6).
    def op(n)
      raise ArgumentError, "AMY::FM: operator must be 1..6" if n < 1 || n > 6
      @ops[n] = Operator.new(self, n) if @ops[n].nil?
      @ops[n]
    end

    # --- Named parameters ---

    def algorithm=(n)
      osc_command(ALGO_OSC, :algorithm, n.to_i)
      @values[:algorithm] = n.to_i
    end

    def feedback=(v)
      osc_command(ALGO_OSC, :feedback, v)
      @values[:feedback] = v
    end

    # :none, :lowpass, :bandpass, :highpass, :lowpass24, :notch
    def filter=(type)
      code = FILTER_TYPES[type]
      raise ArgumentError, "AMY::FM: unknown filter #{type}" if code.nil?
      osc_command(ALGO_OSC, :filter_type, code)
      @filter = type
    end

    def filter
      @filter
    end

    # Filter cutoff in Hz. Turns a lowpass on if no filter is set yet.
    def filter_freq=(hz)
      self.filter = :lowpass if @filter == :none
      osc_command(ALGO_OSC, :filter_freq, hz)
      @values[:filter_freq] = hz
    end

    def resonance=(q)
      osc_command(ALGO_OSC, :resonance, q)
      @values[:resonance] = q
    end

    # Level of this synth (0.0 .. 1.0, 1.0 = unchanged).
    def volume=(v)
      AMY.command(synth: @number, synth_level: v)
      @values[:volume] = v
    end

    # Reverb send level of the output bus (shared by every synth on it).
    def reverb=(v)
      AMY.command(reverb: v)
      @values[:reverb] = v
    end

    def chorus=(v)
      AMY.command(chorus: v)
      @values[:chorus] = v
    end

    def algorithm;   @values[:algorithm];   end
    def feedback;    @values[:feedback];    end
    def filter_freq; @values[:filter_freq]; end
    def resonance;   @values[:resonance];   end
    def volume;      @values[:volume];      end
    def reverb;      @values[:reverb];      end

    # --- Generic set / get ---
    #
    # For handlers that pick the parameter at run time, e.g. from a CC:
    #   fm.set(:attack, 30, op: 2)
    #   fm.set(:filter_freq, 1200)
    # Values are in AMY's units (ms, Hz, 0.0..1.0); see AMY.scale.
    def set(param, value, op: nil)
      return self.op(op).set(param, value) unless op.nil?
      case param
      when :algorithm   then self.algorithm = value
      when :feedback    then self.feedback = value
      when :filter      then self.filter = value
      when :filter_freq then self.filter_freq = value
      when :resonance   then self.resonance = value
      when :volume      then self.volume = value
      when :reverb      then self.reverb = value
      when :chorus      then self.chorus = value
      when :patch       then self.patch = value
      else
        osc_command(ALGO_OSC, param, value)
        @values[param] = value
      end
      value
    end

    # The value last set through this object (nil if not set since the
    # last patch load: AMY is not asked).
    def get(param, op: nil)
      return self.op(op).get(param) unless op.nil?
      return @patch if param == :patch
      return @filter if param == :filter
      @values[param]
    end

    # --- CC mapping (inside AMY) ---
    #
    # Make AMY itself apply CC `cc` on this synth's channel to `param`, with
    # no Ruby in the path. Parameters that cannot be mapped this way (the
    # envelopes, the algorithm, the patch) go through set / the operator
    # accessors instead.
    #   fm.map_cc(74, :filter_freq)                    # default range
    #   fm.map_cc(71, :resonance, min: 0.7, max: 4)
    #   fm.map_cc(21, :level, op: 2)
    def map_cc(cc, param, min: nil, max: nil, log: nil, op: nil)
      target = CC_TARGETS[param]
      raise ArgumentError, "AMY::FM: #{param} cannot be mapped to a CC" if target.nil?
      pnum = target[0]
      lo = min.nil? ? target[2] : min
      hi = max.nil? ? target[3] : max
      lg = log.nil? ? target[4] : log
      spec = [cc, lg ? 1 : 0, lo, hi, 0, pnum]
      case target[1]
      when :algo
        self.filter = :lowpass if param == :filter_freq && @filter == :none
        spec << ALGO_OSC
      when :op
        raise ArgumentError, "AMY::FM: #{param} needs op:" if op.nil?
        spec << self.op(op).osc
      end
      AMY.command(synth: @number, midi_cc: spec)
    end

    # Remove one CC mapping, or all of this synth's mappings.
    def unmap_cc(cc = nil)
      AMY.command(synth: @number, midi_cc: cc.nil? ? 255 : cc)
    end

    # --- Internal ---

    def osc_command(osc, key, value)
      AMY.command_hash({ synth: @number, osc: osc, key => value })
    end

    # ========================================================================
    # One FM operator
    # ========================================================================
    #
    # AMY envelopes are breakpoint lists sent whole, so one ADSR stage cannot
    # be changed on its own. The operator keeps its ADSR here and resends the
    # list on every change. A DX7 preset's envelopes are multi-stage and
    # cannot be read back as ADSR: the first ADSR change on an operator
    # replaces its preset envelope with an ADSR one built from the defaults.
    class Operator
      DEFAULT_ADSR = [10, 300, 0.7, 300]   # attack ms, decay ms, sustain 0..1, release ms

      attr_reader :number

      def initialize(fm, number)
        @fm = fm
        @number = number
        _forget
      end

      # AMY osc (relative to the voice) of this DX7 operator.
      def osc
        8 - @number
      end

      def _forget
        @adsr = nil
        @values = {}
      end

      # Values read back from AMY by FM#refresh. ratio 0 = fixed frequency.
      def _load(level, ratio, breakpoints)
        @values[:level] = level
        @values[:ratio] = ratio if ratio > 0
        a = Operator.adsr_from(breakpoints)
        @adsr = a unless a.nil?
      end

      # "t0,v0,t1,v1,..." (eg0 breakpoints) -> [attack, decay, sustain, release]
      def self.adsr_from(text)
        return nil if text.nil? || text.empty?
        nums = []
        text.split(",").each { |x| nums << x.to_f }
        pairs = []
        i = 0
        while i + 1 < nums.size
          pairs << [nums[i], nums[i + 1]]
          i += 2
        end
        # A DX7 envelope starts with an initial point at time 0, level ~0;
        # it is where the attack starts from, not a stage.
        pairs.shift if pairs.size > 2 && pairs[0][0] == 0 && pairs[0][1] < 0.01
        return nil if pairs.size < 2
        decay = 0.0
        j = 1
        while j < pairs.size - 1
          decay += pairs[j][0]
          j += 1
        end
        [pairs[0][0], decay, pairs[pairs.size - 2][1], pairs[pairs.size - 1][0]]
      end

      # Frequency as a ratio of the note frequency.
      def ratio=(r)
        command(:ratio, r)
        @values[:ratio] = r
      end

      # Output level (0.0 .. 1.0): for a modulator, the modulation depth.
      def level=(l)
        command(:amp, l)   # first coefficient only: the envelope etc. stay
        @values[:level] = l
      end

      def attack=(ms);  set_adsr(0, ms); end
      def decay=(ms);   set_adsr(1, ms); end
      def sustain=(v);  set_adsr(2, v);  end
      def release=(ms); set_adsr(3, ms); end

      def attack;  adsr[0]; end
      def decay;   adsr[1]; end
      def sustain; adsr[2]; end
      def release; adsr[3]; end
      def ratio;   @values[:ratio]; end
      def level;   @values[:level]; end

      # Several stages in one message.
      def envelope(attack: nil, decay: nil, sustain: nil, release: nil)
        a = adsr.dup
        a[0] = attack unless attack.nil?
        a[1] = decay unless decay.nil?
        a[2] = sustain unless sustain.nil?
        a[3] = release unless release.nil?
        @adsr = a
        send_envelope
      end

      def set(param, value)
        case param
        when :ratio   then self.ratio = value
        when :level   then self.level = value
        when :attack  then self.attack = value
        when :decay   then self.decay = value
        when :sustain then self.sustain = value
        when :release then self.release = value
        else
          command(param, value)
          @values[param] = value
        end
        value
      end

      def get(param)
        case param
        when :attack  then attack
        when :decay   then decay
        when :sustain then sustain
        when :release then release
        else @values[param]
        end
      end

      private

      def adsr
        @adsr.nil? ? DEFAULT_ADSR : @adsr
      end

      def set_adsr(index, value)
        a = adsr.dup
        a[index] = value
        @adsr = a
        send_envelope
      end

      def send_envelope
        a = adsr
        # attack to 1.0, decay to the sustain level, release to 0
        command(:bp0, [a[0], 1, a[1], a[2], a[3], 0])
      end

      def command(key, value)
        @fm.osc_command(osc, key, value)
      end
    end
  end
end
