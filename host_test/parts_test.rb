# Host tests for AMY's parts (AMY.add_fm / AMY.add_synth, mrblib/amy.rb and
# mrblib/amy_synth.rb): synth numbers, channel bindings, removal, and the
# commands a part sends. The C side is stubbed: the wire messages are
# recorded and AMY._bind keeps a channel -> synths table like amy_port.c.
#
#   ruby host_test/parts_test.rb    (from the gem directory)
#
# mruby has no `load`; concatenate the stubs, the mrblib and the tests:
#
#   { sed -n '/^# -- stubs/,/^# -- end stubs/p' host_test/parts_test.rb
#     cat mrblib/amy.rb mrblib/amy_synth.rb mrblib/patch_names.rb
#     sed -n '/^\$passed = 0/,$p' host_test/parts_test.rb; } | RBENV_VERSION=mruby-3.2.0 mruby

# -- stubs
module AMY
  $wire = []
  $bind = Array.new(16, 0)   # channel -> mask, bit n - 1 = synth n
  $blocks = 0

  def self._start; true; end
  def self._running; true; end
  def self._reset
    $bind = Array.new(16, 0)
  end
  def self._send(msg)
    $wire << msg
    0
  end
  def self._blocks
    $blocks += 1
  end
  def self._bind(synth, channel)
    return -1 if synth < 1 || synth > 16 || channel > 15
    bit = 1 << (synth - 1)
    16.times do |ch|
      $bind[ch] = ch == channel ? ($bind[ch] | bit) : ($bind[ch] & ~bit)
    end
    0
  end
  def self._channel_synths(channel)
    $bind[channel]
  end
  def self._transport_bit; 0x08; end
  def self._fm_state(_n); nil; end
  def self._synth_state(_n); nil; end
  def self._log_scale(t, lo, hi); lo * (hi / lo)**t; end
  def self._log_unscale(x, lo, hi); Math.log(x / lo) / Math.log(hi / lo); end
end

def sleep_ms(_ms); end
# -- end stubs

dir = File.expand_path("..", File.dirname(__FILE__))
load File.join(dir, "mrblib/amy.rb")
load File.join(dir, "mrblib/amy_synth.rb")
load File.join(dir, "mrblib/patch_names.rb")

$passed = 0
$failed = 0

def check(name, expected, actual)
  if expected == actual
    $passed += 1
  else
    $failed += 1
    puts "FAIL #{name}: expected #{expected.inspect}, got #{actual.inspect}"
  end
end

def raises(name)
  yield
  $failed += 1
  puts "FAIL #{name}: no exception"
rescue => e
  $passed += 1
  e
end

amy = AMY::Transport.instance

# --- numbers are allocated, not derived from the channel ---
fm = amy.add_fm(channel: 3, voices: 4, patch: 130)
check "fm kind", :fm, fm.kind
check "fm first synth", 1, fm.synth_number
check "fm channel", 3, fm.channel
check "fm bound", [1], AMY.channel_synths(3)
check "fm patch load addressed to its synth", true, $wire.include?("i1iv4K130Z")

syn = AMY.add_synth(channel: 3, voices: 2)
check "synth kind", :synth, syn.kind
check "synth second number", 2, syn.synth_number
check "layer on one channel", [1, 2], AMY.channel_synths(3)
check "init voice addressed to synth 2", true, $wire.include?("i2iv2in6Z")
check "no command without a synth", [], $wire.reject { |m| m.start_with?("i") }

check "parts", [fm, syn], AMY.parts
check "parts on channel", [fm, syn], amy.parts(channel: 3)
check "parts on another channel", [], AMY.parts(channel: 0)

# --- parameters go to the part's own synth ---
$wire.clear
syn.cutoff = 1200
check "cutoff on synth 2 head", ["i2v0F1200Z"], $wire
$wire.clear
fm.volume = 0.5
check "volume on synth 1", ["i1iV0.5Z"], $wire
check "volume read back", 0.5, fm.volume
$wire.clear
syn.map_cc(74, :cutoff)
check "map_cc on synth 2", true, $wire[0].start_with?("i2ic74,")

# --- moving a part to another channel ---
fm.channel = 5
check "moved: old channel", [2], AMY.channel_synths(3)
check "moved: new channel", [1], AMY.channel_synths(5)
fm.channel = nil
check "no channel", [], AMY.channel_synths(5)
check "channel nil", nil, fm.channel
raises("channel out of range") { fm.channel = 16 }

# --- removal frees the number for the next part ---
$wire.clear
check "remove", true, fm.remove
check "removed?", true, fm.removed?
check "remove again", false, fm.remove
check "removal deletes the synth", ["i1iv0Z"], $wire
check "parts after remove", [syn], AMY.parts
raises("command on a removed part") { fm.algorithm = 3 }
fm2 = AMY.add_fm(channel: 0)
check "number reused", 1, fm2.synth_number
check "reused number bound", [1], AMY.channel_synths(0)

# --- running out ---
extra = []
14.times { |i| extra << AMY.add_fm(channel: i % 16) }
check "sixteen parts", 16, AMY.parts.size
e = raises("seventeenth part") { AMY.add_fm }
check "out of parts message", true, e.message.include?("no free synth")

# --- reset forgets every part ---
AMY.reset
check "no parts after reset", [], AMY.parts
check "reset part is removed", true, syn.removed?
check "bindings cleared", [], AMY.channel_synths(3)
again = AMY.add_synth(channel: 9, patch: 1)
check "numbering restarts", 1, again.synth_number

# --- the old constructors are the same thing ---
old = AMY::FM.new(channel: 2, voices: 2, patch: 128)
check "FM.new registers", true, AMY.parts.include?(old)
check "FM.new number", 2, old.synth_number

# --- voices= reloads ---
$wire.clear
old.voices = 6
check "voices reload", true, $wire.include?("i2iv6K128Z")

# --- low-level bind ---
check "bind", true, AMY.bind(16, 4)
check "bound", [16], AMY.channel_synths(4)
check "unbind", true, AMY.bind(16, nil)

AMY.remove_all
check "remove_all", [], AMY.parts

puts "#{$passed} passed, #{$failed} failed"
raise "#{$failed} failed" if $failed > 0
