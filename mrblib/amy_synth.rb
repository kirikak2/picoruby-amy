# AMY::Synth -- a subtractive synth with up to four oscillators, one LFO,
# a filter and two ADSR envelopes, on one MIDI channel.
#
#   syn = AMY::Synth.new(channel: 1, voices: 6)    # channel 1 = AMY synth 2
#   syn.patch = 1                     # start from a Juno-60 preset (0..127)
#   syn.osc(1).wave = :saw
#   syn.osc(2).octave = -1
#   syn.cutoff = 1200
#   syn.filter_envelope(attack: 5, decay: 400, sustain: 0.3, release: 300)
#   syn.lfo.vibrato = 0.02
#   dev = MIDI::Device.new(syn.transport)
#
# The voice is laid out the way AMY's Juno patches are, so any of them loads
# into it and reads back onto the same parameters:
#
#   osc 0  SILENT head: the filter and the amplifier (VCF / VCA) of the voice.
#          bp0 is the amp envelope, bp1 the filter envelope.
#   osc 1  the LFO. Not chained, so it is not heard; the others name it as
#          their mod_source and scale it with their mod0 coefficients.
#   osc 2..5  oscillators 1..4, chained 0 -> 2 -> 3 -> 4 -> 5. A chain sums
#          into the head, whose filter, amp envelope, velocity and pan apply
#          once to the sum; the members' own velocity and envelope terms are
#          off. An unused oscillator just has level 0.
#
# Parameters set here are commands to the whole synth, so they reach every
# voice at once.
module AMY
  class Synth
    HEAD = 0
    LFO_OSC = 1
    FIRST_OSC = 2
    MAX_OSCS = 4
    OSCS_PER_VOICE = FIRST_OSC + MAX_OSCS
    WAVE_SILENT = 20

    # AMY wave numbers. :saw is AMY's SAW_UP, the one the Juno patches use.
    WAVES = {
      sine: 0, pulse: 1, saw_down: 2, saw: 3, triangle: 4, noise: 5,
      ks: 6, pcm: 7
    }
    # The waves a knob steps through (KS and PCM need more setup).
    WAVE_LIST = [:sine, :triangle, :saw, :saw_down, :pulse, :noise]

    FILTER_TYPES = {
      none: 0, lowpass: 1, bandpass: 2, highpass: 3, lowpass24: 4,
      notch: 5, phaser: 6
    }
    FILTER_LIST = [:none, :lowpass, :lowpass24, :bandpass, :highpass, :notch, :phaser]

    LFO_WAVE_LIST = [:sine, :triangle, :saw, :saw_down, :pulse, :noise]

    DEFAULT_AMP_ADSR    = [5, 200, 0.8, 300]   # attack ms, decay ms, sustain 0..1, release ms
    DEFAULT_FILTER_ADSR = [5, 400, 0.3, 300]

    # Octave / detune go into the oscillator's frequency constant, which is
    # in Hz with 440 meaning "the played note". The log mapping helpers
    # cover +-FREQ_SPAN_OCT octaves around it.
    BASE_FREQ = 440.0
    FREQ_SPAN_OCT = 4.0

    # ControlCoefficient slots (AMY: const, note, vel, eg0, eg1, mod0, bend).
    C_CONST = 0
    C_NOTE  = 1
    C_VEL   = 2
    C_EG0   = 3
    C_EG1   = 4
    C_MOD0  = 5
    C_BEND  = 6

    # param => [AMY PARAM number, target, min, max, log]
    #   :head  the filter / amp osc   :lfo  the LFO osc
    #   :osc   one oscillator (osc: n), or all four when osc: is omitted
    #   :bus   the output bus
    CC_TARGETS = {
      cutoff:     [PARAM_FILTER_FREQ,  :head, 100,  8000, true],
      resonance:  [PARAM_RESONANCE,    :head, 0.7,  8.0,  false],
      pan:        [PARAM_PAN,          :head, 0.0,  1.0,  false],
      glide:      [PARAM_PORTAMENTO,   :osc,  0,    500,  false],
      level:      [PARAM_AMP,          :osc,  0.0,  1.0,  false],
      duty:       [PARAM_DUTY,         :osc,  0.05, 0.95, false],
      lfo_rate:   [PARAM_FREQ,         :lfo,  0.1,  20.0, true],
      volume:     [PARAM_VOLUME,       :bus,  0.0,  1.0,  false],
      reverb:     [PARAM_REVERB_LEVEL, :bus,  0.0,  1.0,  false],
      chorus:     [PARAM_CHORUS_LEVEL, :bus,  0.0,  1.0,  false],
      echo:       [PARAM_ECHO_LEVEL,   :bus,  0.0,  1.0,  false]
    }

    attr_reader :channel, :voices, :patch, :transport, :lfo

    # channel: MIDI channel, 0-based (AMY synth channel + 1)
    # voices:  polyphony
    # patch:   a Juno-60 preset (0..127) to start from, or nil for the
    #          plain initial voice (one saw through an open lowpass)
    def initialize(channel: 0, voices: 6, patch: nil)
      @transport = AMY::Transport.instance
      @channel = channel
      @number = channel + 1
      @voices = voices
      @oscs = []
      i = 1
      while i <= MAX_OSCS
        @oscs << Oscillator.new(self, i)
        i += 1
      end
      @lfo = LFO.new(self)
      @values = {}
      @amp_adsr = DEFAULT_AMP_ADSR.dup
      @filter_adsr = DEFAULT_FILTER_ADSR.dup
      if patch.nil?
        init_voice
      else
        self.patch = patch
      end
    end

    # The AMY synth number this object drives (channel + 1).
    def synth_number
      @number
    end

    # Oscillator n (1..4).
    def osc(n)
      raise ArgumentError, "AMY::Synth: oscillator must be 1..#{MAX_OSCS}" if n < 1 || n > MAX_OSCS
      @oscs[n - 1]
    end

    def oscs
      @oscs
    end

    # --- Voice setup ---

    # Build the plain initial voice: oscillator 1 a saw, 2 a pulse, 3 a
    # pulse an octave down, 4 noise (2..4 at level 0), through an open
    # 24 dB lowpass, LFO a 5 Hz triangle routed nowhere.
    def init_voice
      AMY.command(synth: @number, num_voices: @voices, oscs_per_voice: OSCS_PER_VOICE)
      AMY.command(synth: @number, synth_flags: SYNTH_FLAGS_NO_NOTE_WARNINGS)
      @patch = nil
      @values = {
        filter: :lowpass24, cutoff: 4000.0, resonance: 0.7, key_track: 0.0,
        filter_env: 0.0, velocity: 1.0, pan: 0.5, glide: 0, volume: 1.0
      }
      @amp_adsr = DEFAULT_AMP_ADSR.dup
      @filter_adsr = DEFAULT_FILTER_ADSR.dup
      @vca = 1.0

      head({
        wave: WAVE_SILENT, chained_osc: FIRST_OSC, mod_source: LFO_OSC,
        filter_type: FILTER_TYPES[:lowpass24], resonance: 0.7,
        filter_freq: [4000, 0, 0, 0, 0, 0, 0],
        amp: [1, 0, 1, 1, 0, 0, 0], pan: [0.5, 0, 0, 0, 0, 0, 0],
        eg0_type: 0, eg1_type: 0
      })
      send_amp_envelope
      send_filter_envelope

      @lfo._init
      defaults = [[:saw, 0.8, 0], [:pulse, 0.0, 0], [:pulse, 0.0, -1], [:noise, 0.0, 0]]
      i = 0
      while i < MAX_OSCS
        d = defaults[i]
        @oscs[i]._init(d[0], d[1], d[2], i < MAX_OSCS - 1 ? FIRST_OSC + i + 1 : nil)
        i += 1
      end
      self
    end

    # Load a Juno-60 preset (0..127) and read it back, so every parameter
    # here reports the preset. Takes a couple of audio blocks (~12 ms).
    #
    # A Juno has one envelope, shared by the VCA and the VCF; its patches
    # drive the filter from the amp envelope (the filter_freq eg0 term).
    # Here the filter has its own (bp1 / eg1), so after the load the amp
    # envelope is copied to the filter envelope and the depth moved to eg1:
    # the preset sounds the same, and the two can then be changed apart.
    def patch=(number)
      if number < 0 || number > 127
        raise ArgumentError, "AMY::Synth: patch must be a Juno preset, 0..127"
      end
      AMY.command(synth: @number, num_voices: @voices, patch: number)
      AMY.command(synth: @number, synth_flags: SYNTH_FLAGS_NO_NOTE_WARNINGS)
      @patch = number
      refresh
      split_filter_envelope
      number
    end

    # The preset's name ("A12 Brass Swell"), or nil for the initial voice.
    def patch_name
      @patch.nil? ? nil : AMY.patch_name(@patch)
    end

    # Read the synth back from AMY into the values reported here. Waits two
    # audio blocks first, so that a patch load has landed.
    # @return [Boolean] false if AMY has no such synth
    def refresh
      AMY.wait_blocks(2)
      text = AMY._synth_state(@number)
      return false if text.nil?
      oscs = {}
      fx = {}
      text.split("\n").each do |line|
        f = line.split(" ")
        if f[0] == "synth"
          @values[:volume] = f[3].to_f
        elsif f[0] == "osc"
          oscs[f[1].to_i] = Synth.fields(f, 2)
        elsif f[0] == "fx"
          fx = Synth.fields(f, 1)
        end
      end
      _load_head(oscs[HEAD] || {})
      @lfo._load(oscs[LFO_OSC] || {}, oscs[FIRST_OSC] || {}, oscs[HEAD] || {})
      i = 0
      while i < MAX_OSCS
        @oscs[i]._load(oscs[FIRST_OSC + i] || {})
        i += 1
      end
      @values[:glide] = @oscs[0].glide_ms
      @values[:reverb] = Synth.coef(fx["h"], 0, 0.0)
      @values[:chorus] = Synth.coef(fx["k"], 0, 0.0)
      @values[:echo_level] = Synth.coef(fx["M"], 0, 0.0)
      @values[:echo_delay] = Synth.coef(fx["M"], 1, 500.0)
      @values[:echo_feedback] = Synth.coef(fx["M"], 3, 0.0)
      true
    end

    # --- Filter (on the head) ---

    # :none :lowpass :lowpass24 :bandpass :highpass :notch :phaser
    def filter=(type)
      code = FILTER_TYPES[type]
      raise ArgumentError, "AMY::Synth: unknown filter #{type}" if code.nil?
      head_command(:filter_type, code)
      @values[:filter] = type
    end

    # Cutoff in Hz (before key tracking, envelope and LFO).
    def cutoff=(hz)
      head_command(:filter_freq, [hz])
      @values[:cutoff] = hz
    end

    def resonance=(q)
      head_command(:resonance, q)
      @values[:resonance] = q
    end

    # How far the filter envelope moves the cutoff, in octaves (may be < 0).
    def filter_env=(octaves)
      head_command(:filter_freq, [nil, nil, nil, 0, octaves])
      @values[:filter_env] = octaves
    end

    # Cutoff following the played note: 1.0 = in step with the pitch.
    def key_track=(amount)
      head_command(:filter_freq, [nil, amount])
      @values[:key_track] = amount
    end

    # --- Amplifier (on the head) ---

    # Velocity sensitivity (0 = every note at full level).
    def velocity=(v)
      head_command(:amp, [nil, nil, v])
      @values[:velocity] = v
    end

    # Stereo position of the whole voice, 0.0 (left) .. 1.0 (right).
    def pan=(v)
      head_command(:pan, [v])
      @values[:pan] = v
    end

    # Level of this synth (0.0 .. 1.0, 1.0 = unchanged).
    def volume=(v)
      AMY.command(synth: @number, synth_level: v)
      @values[:volume] = v
    end

    # Portamento of every oscillator, in ms.
    def glide=(ms)
      @oscs.each { |o| o._command(:portamento, ms.to_i) }
      @values[:glide] = ms
    end

    # --- Envelopes (ADSR) ---

    def amp_envelope(attack: nil, decay: nil, sustain: nil, release: nil)
      Synth.merge_adsr(@amp_adsr, attack, decay, sustain, release)
      send_amp_envelope
    end

    def filter_envelope(attack: nil, decay: nil, sustain: nil, release: nil)
      Synth.merge_adsr(@filter_adsr, attack, decay, sustain, release)
      send_filter_envelope
    end

    def amp_attack=(ms);     amp_envelope(attack: ms);     end
    def amp_decay=(ms);      amp_envelope(decay: ms);      end
    def amp_sustain=(v);     amp_envelope(sustain: v);     end
    def amp_release=(ms);    amp_envelope(release: ms);    end
    def filter_attack=(ms);  filter_envelope(attack: ms);  end
    def filter_decay=(ms);   filter_envelope(decay: ms);   end
    def filter_sustain=(v);  filter_envelope(sustain: v);  end
    def filter_release=(ms); filter_envelope(release: ms); end

    def amp_attack;     @amp_adsr[0];    end
    def amp_decay;      @amp_adsr[1];    end
    def amp_sustain;    @amp_adsr[2];    end
    def amp_release;    @amp_adsr[3];    end
    def filter_attack;  @filter_adsr[0]; end
    def filter_decay;   @filter_adsr[1]; end
    def filter_sustain; @filter_adsr[2]; end
    def filter_release; @filter_adsr[3]; end

    # --- Effects (on the output bus, shared by every synth on it) ---

    def reverb=(v)
      AMY.command(reverb: v)
      @values[:reverb] = v
    end

    def chorus=(v)
      AMY.command(chorus: v)
      @values[:chorus] = v
    end

    # Echo: level 0.0..1.0, delay in ms (up to 743), feedback 0.0..1.0.
    # The delay line is allocated (in PSRAM) when the level first goes
    # above 0.
    def echo(level: nil, delay: nil, feedback: nil)
      AMY.command(echo: [level, delay, nil, feedback])
      @values[:echo_level] = level unless level.nil?
      @values[:echo_delay] = delay unless delay.nil?
      @values[:echo_feedback] = feedback unless feedback.nil?
      nil
    end

    def echo_level=(v);    echo(level: v);    end
    def echo_delay=(ms);   echo(delay: ms);   end
    def echo_feedback=(v); echo(feedback: v); end

    # --- Readers ---

    def filter;        @values[:filter];        end
    def cutoff;        @values[:cutoff];        end
    def resonance;     @values[:resonance];     end
    def filter_env;    @values[:filter_env];    end
    def key_track;     @values[:key_track];     end
    def velocity;      @values[:velocity];      end
    def pan;           @values[:pan];           end
    def volume;        @values[:volume];        end
    def glide;         @values[:glide];         end
    def reverb;        @values[:reverb];        end
    def chorus;        @values[:chorus];        end
    def echo_level;    @values[:echo_level];    end
    def echo_delay;    @values[:echo_delay];    end
    def echo_feedback; @values[:echo_feedback]; end

    # --- Generic set / get ---
    #
    # For handlers that pick the parameter at run time:
    #   syn.set(:cutoff, 1200)
    #   syn.set(:octave, -1, osc: 2)
    #   syn.set(:rate, 6.0, lfo: true)
    # Values are in AMY's units (Hz, ms, 0.0..1.0); see AMY.scale.
    def set(param, value, osc: nil, lfo: false)
      return self.osc(osc).set(param, value) unless osc.nil?
      return @lfo.set(param, value) if lfo
      case param
      when :patch          then self.patch = value
      when :filter         then self.filter = value
      when :cutoff         then self.cutoff = value
      when :resonance      then self.resonance = value
      when :filter_env     then self.filter_env = value
      when :key_track      then self.key_track = value
      when :velocity       then self.velocity = value
      when :pan            then self.pan = value
      when :volume         then self.volume = value
      when :glide          then self.glide = value
      when :reverb         then self.reverb = value
      when :chorus         then self.chorus = value
      when :echo_level     then self.echo_level = value
      when :echo_delay     then self.echo_delay = value
      when :echo_feedback  then self.echo_feedback = value
      when :amp_attack     then self.amp_attack = value
      when :amp_decay      then self.amp_decay = value
      when :amp_sustain    then self.amp_sustain = value
      when :amp_release    then self.amp_release = value
      when :filter_attack  then self.filter_attack = value
      when :filter_decay   then self.filter_decay = value
      when :filter_sustain then self.filter_sustain = value
      when :filter_release then self.filter_release = value
      else
        raise ArgumentError, "AMY::Synth: unknown parameter #{param}"
      end
      value
    end

    def get(param, osc: nil, lfo: false)
      return self.osc(osc).get(param) unless osc.nil?
      return @lfo.get(param) if lfo
      case param
      when :patch          then @patch
      when :amp_attack     then amp_attack
      when :amp_decay      then amp_decay
      when :amp_sustain    then amp_sustain
      when :amp_release    then amp_release
      when :filter_attack  then filter_attack
      when :filter_decay   then filter_decay
      when :filter_sustain then filter_sustain
      when :filter_release then filter_release
      else @values[param]
      end
    end

    # --- CC mapping (inside AMY) ---
    #
    # Make AMY itself apply CC `cc` on this synth's channel to `param`, with
    # no Ruby in the path (see CC_TARGETS for what can be mapped).
    #   syn.map_cc(74, :cutoff)                       # default range
    #   syn.map_cc(71, :resonance, min: 0.7, max: 4)
    #   syn.map_cc(70, :duty, osc: 2)                 # one oscillator
    #   syn.map_cc(5, :glide)                         # all four
    def map_cc(cc, param, min: nil, max: nil, log: nil, osc: nil)
      target = CC_TARGETS[param]
      raise ArgumentError, "AMY::Synth: #{param} cannot be mapped to a CC" if target.nil?
      pnum = target[0]
      lo = min.nil? ? target[2] : min
      hi = max.nil? ? target[3] : max
      lg = log.nil? ? target[4] : log
      spec = [cc, lg ? 1 : 0, lo, hi, 0]
      case target[1]
      when :head
        spec << pnum << HEAD
      when :lfo
        spec << pnum << LFO_OSC
      when :osc
        if osc.nil?
          @oscs.each { |o| spec << pnum << o.osc }   # up to 4 pairs per CC
        else
          spec << pnum << self.osc(osc).osc
        end
      else
        spec << pnum
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

    def head_command(key, value)
      osc_command(HEAD, key, value)
    end

    # Several parameters of the head in one message.
    def head(params)
      h = { synth: @number, osc: HEAD }
      params.each { |k, v| h[k] = v }
      AMY.command_hash(h)
    end

    # "w=1 a=0.5,,0" fields from index `from` -> { "w" => "1", "a" => "0.5,,0" }
    def self.fields(words, from)
      h = {}
      i = from
      while i < words.size
        kv = words[i].split("=")
        h[kv[0]] = kv[1] if kv.size == 2
        i += 1
      end
      h
    end

    # Slot `i` of a comma list (a ControlCoefficient list as read back), or
    # `default` when the list or the slot is absent / empty.
    def self.coef(list, i, default)
      return default if list.nil?
      parts = list.split(",")
      return default if i >= parts.size || parts[i].empty?
      parts[i].to_f
    end

    def self.merge_adsr(adsr, attack, decay, sustain, release)
      adsr[0] = attack unless attack.nil?
      adsr[1] = decay unless decay.nil?
      adsr[2] = sustain unless sustain.nil?
      adsr[3] = release unless release.nil?
      adsr
    end

    # [attack, decay, sustain, release] -> AMY breakpoints: attack to 1.0,
    # decay to the sustain level, release to 0.
    def self.adsr_bps(a)
      [a[0], 1, a[1], a[2], a[3], 0]
    end

    # Hz <-> octaves from BASE_FREQ, within +-FREQ_SPAN_OCT (no Math here).
    def self.octaves_to_freq(oct)
      lo = BASE_FREQ / 16.0
      AMY._log_scale((oct + FREQ_SPAN_OCT) / (2 * FREQ_SPAN_OCT), lo, BASE_FREQ * 16.0)
    end

    def self.freq_to_octaves(hz)
      lo = BASE_FREQ / 16.0
      AMY._log_unscale(hz, lo, BASE_FREQ * 16.0) * (2 * FREQ_SPAN_OCT) - FREQ_SPAN_OCT
    end

    def self.round(x)
      x >= 0 ? (x + 0.5).to_i : -((-x + 0.5).to_i)
    end

    def send_amp_envelope
      head_command(:bp0, Synth.adsr_bps(@amp_adsr))
    end

    def send_filter_envelope
      head_command(:bp1, Synth.adsr_bps(@filter_adsr))
    end

    def _load_head(h)
      code = (h["G"] || "0").to_i
      @values[:filter] = :none
      FILTER_TYPES.each { |name, c| @values[:filter] = name if c == code }
      @values[:resonance] = h["R"].nil? ? 0.7 : h["R"].to_f
      @values[:cutoff] = Synth.coef(h["F"], C_CONST, BASE_FREQ)
      @values[:key_track] = Synth.coef(h["F"], C_NOTE, 0.0)
      @values[:filter_env] = Synth.coef(h["F"], C_EG1, 0.0)
      @juno_filter_env = Synth.coef(h["F"], C_EG0, 0.0)
      @values[:velocity] = Synth.coef(h["a"], C_VEL, 1.0)
      @values[:pan] = Synth.coef(h["Q"], C_CONST, 0.5)
      @vca = Synth.coef(h["a"], C_CONST, 1.0)
      @amp_bps = h["A"]
      a = AMY::FM::Operator.adsr_from(h["A"])
      @amp_adsr = a.nil? ? [0, 0, 1.0, 0] : a
      f = AMY::FM::Operator.adsr_from(h["B"])
      @filter_adsr = f.nil? ? DEFAULT_FILTER_ADSR.dup : f
    end

    # See patch=: a Juno patch's filter rides the amp envelope (eg0).
    def split_filter_envelope
      depth = @juno_filter_env
      return if depth.nil? || depth == 0
      head_command(:filter_freq, [nil, nil, nil, 0, depth])
      # The preset's own breakpoints, so the sound is unchanged.
      head_command(:bp1, @amp_bps) unless @amp_bps.nil?
      @filter_adsr = @amp_adsr.dup
      @values[:filter_env] = depth
      @juno_filter_env = 0.0
    end

    # ========================================================================
    # One oscillator (osc 2..5 of the voice)
    # ========================================================================
    class Oscillator
      attr_reader :number

      def initialize(synth, number)
        @synth = synth
        @number = number
        @values = { wave: :saw, level: 0.0, octave: 0, detune: 0.0, duty: 0.5, glide: 0 }
      end

      # AMY osc (relative to the voice).
      def osc
        FIRST_OSC + @number - 1
      end

      # :sine :triangle :saw :saw_down :pulse :noise (:ks :pcm), or an AMY
      # wave number.
      def wave=(w)
        code = w.is_a?(Integer) ? w : WAVES[w]
        raise ArgumentError, "AMY::Synth: unknown wave #{w}" if code.nil?
        _command(:wave, code)
        @values[:wave] = Oscillator.wave_name(code)
      end

      def level=(l)
        _command(:amp, [l])   # const only: velocity / envelope stay off
        @values[:level] = l
      end

      # Whole octaves up / down from the played note.
      def octave=(n)
        @values[:octave] = n.to_i
        send_freq
      end

      # Fine tuning in cents (-100..100).
      def detune=(cents)
        @values[:detune] = cents
        send_freq
      end

      # Pulse width (0.0..1.0) of a pulse wave.
      def duty=(d)
        _command(:duty, [d])
        @values[:duty] = d
      end

      # PCM sample / KS preset.
      def preset=(n)
        _command(:preset, n)
        @values[:preset] = n
      end

      def wave;   @values[:wave];   end
      def level;  @values[:level];  end
      def octave; @values[:octave]; end
      def detune; @values[:detune]; end
      def duty;   @values[:duty];   end
      def glide_ms; @values[:glide]; end

      def set(param, value)
        case param
        when :wave   then self.wave = value
        when :level  then self.level = value
        when :octave then self.octave = value
        when :detune then self.detune = value
        when :duty   then self.duty = value
        when :preset then self.preset = value
        else
          raise ArgumentError, "AMY::Synth: unknown oscillator parameter #{param}"
        end
        value
      end

      def get(param)
        @values[param]
      end

      def self.wave_name(code)
        name = nil
        WAVES.each { |k, c| name = k if c == code && name.nil? }
        name.nil? ? code : name
      end

      # --- Internal ---

      def _command(key, value)
        @synth.osc_command(osc, key, value)
      end

      def _init(wave, level, octave, chained)
        @values = { wave: wave, level: level, octave: octave, detune: 0.0, duty: 0.5, glide: 0 }
        h = {
          synth: @synth.synth_number, osc: osc, wave: WAVES[wave],
          amp: [level, 0, 0, 0, 0, 0, 0],
          freq: [Synth.octaves_to_freq(octave.to_f), 1, 0, 0, 0, 0, 1],
          duty: [0.5, 0, 0, 0, 0, 0, 0],
          mod_source: LFO_OSC, portamento: 0
        }
        h[:chained_osc] = chained unless chained.nil?
        AMY.command_hash(h)
      end

      def _load(h)
        @values[:wave] = Oscillator.wave_name((h["w"] || "0").to_i)
        @values[:level] = Synth.coef(h["a"], C_CONST, 1.0)
        total = Synth.freq_to_octaves(Synth.coef(h["f"], C_CONST, BASE_FREQ))
        oct = Synth.round(total)
        @values[:octave] = oct
        @values[:detune] = Synth.round((total - oct) * 1200.0)
        @values[:duty] = Synth.coef(h["d"], C_CONST, 0.5)
        @values[:glide] = (h["m"] || "0").to_i
      end

      private

      def send_freq
        oct = @values[:octave] + @values[:detune] / 1200.0
        _command(:freq, [Synth.octaves_to_freq(oct)])
      end
    end

    # ========================================================================
    # The LFO (osc 1) and where it goes
    # ========================================================================
    class LFO
      def initialize(synth)
        @synth = synth
        @values = { wave: :triangle, rate: 5.0, vibrato: 0.0, filter: 0.0, tremolo: 0.0, pwm: 0.0 }
      end

      # :sine :triangle :saw :saw_down :pulse :noise (noise = sample & hold-ish)
      def wave=(w)
        code = w.is_a?(Integer) ? w : WAVES[w]
        raise ArgumentError, "AMY::Synth: unknown wave #{w}" if code.nil?
        command(LFO_OSC, :wave, code)
        @values[:wave] = Oscillator.wave_name(code)
      end

      # Speed in Hz.
      def rate=(hz)
        command(LFO_OSC, :freq, [hz])
        @values[:rate] = hz
      end

      # Pitch modulation of every oscillator, in octaves (0.02 ~ 24 cents).
      def vibrato=(octaves)
        @synth.oscs.each { |o| o._command(:freq, [nil, nil, nil, nil, nil, octaves]) }
        @values[:vibrato] = octaves
      end

      # Cutoff modulation, in octaves.
      def filter=(octaves)
        command(HEAD, :filter_freq, [nil, nil, nil, nil, nil, octaves])
        @values[:filter] = octaves
      end

      # Level modulation of the voice.
      def tremolo=(amount)
        command(HEAD, :amp, [nil, nil, nil, nil, nil, amount])
        @values[:tremolo] = amount
      end

      # Pulse-width modulation of every oscillator (0.0..0.5).
      def pwm=(amount)
        @synth.oscs.each { |o| o._command(:duty, [nil, nil, nil, nil, nil, amount]) }
        @values[:pwm] = amount
      end

      def wave;    @values[:wave];    end
      def rate;    @values[:rate];    end
      def vibrato; @values[:vibrato]; end
      def filter;  @values[:filter];  end
      def tremolo; @values[:tremolo]; end
      def pwm;     @values[:pwm];     end

      def set(param, value)
        case param
        when :wave    then self.wave = value
        when :rate    then self.rate = value
        when :vibrato then self.vibrato = value
        when :filter  then self.filter = value
        when :tremolo then self.tremolo = value
        when :pwm     then self.pwm = value
        else
          raise ArgumentError, "AMY::Synth: unknown LFO parameter #{param}"
        end
        value
      end

      def get(param)
        @values[param]
      end

      # --- Internal ---

      # The LFO's own pitch ignores the note and the bend, and its level
      # ignores velocity and the envelope, so it runs the same whether or
      # not a note reached it.
      def _init
        @values = { wave: :triangle, rate: 5.0, vibrato: 0.0, filter: 0.0, tremolo: 0.0, pwm: 0.0 }
        AMY.command_hash({
          synth: @synth.synth_number, osc: LFO_OSC, wave: WAVES[:triangle],
          freq: [5.0, 0, 0, 0, 0, 0, 0], amp: [1, 0, 0, 0, 0, 0, 0],
          bp0: [0, 1, 0, 0]
        })
      end

      # lfo: osc 1, first: oscillator 1 (vibrato, PWM), head: osc 0
      def _load(lfo, first, head)
        @values[:wave] = Oscillator.wave_name((lfo["w"] || "0").to_i)
        @values[:rate] = Synth.coef(lfo["f"], C_CONST, BASE_FREQ)
        @values[:vibrato] = Synth.coef(first["f"], C_MOD0, 0.0)
        @values[:pwm] = Synth.coef(first["d"], C_MOD0, 0.0)
        @values[:filter] = Synth.coef(head["F"], C_MOD0, 0.0)
        @values[:tremolo] = Synth.coef(head["a"], C_MOD0, 0.0)
      end

      private

      def command(osc, key, value)
        @synth.osc_command(osc, key, value)
      end
    end
  end
end
