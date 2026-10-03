# Play a DX7 preset on AMY and change it while it plays.
require 'midi'
require 'amy'

fm  = AMY::FM.new(channel: 0, voices: 4, patch: 130)   # DX7 "BRASS 3"
dev = MIDI::Device.new(fm.synth)

[60, 64, 67, 72].each do |note|
  dev.trigger(note, 100, duration: 300)
  sleep_ms 350
end

# Brighter: more feedback, a slower attack on operator 1, some reverb
fm.feedback = 0.6
fm.op(1).attack = 150
fm.reverb = 0.3
dev.trigger(48, 110, duration: 1200)
sleep_ms 1500

# Let CC 74 open and close a lowpass filter, from any MIDI source
fm.map_cc(74, :filter_freq, min: 200, max: 6000)
[127, 80, 40, 10].each do |v|
  dev.control_change(74, v)
  dev.trigger(55, 100, duration: 250)
  sleep_ms 300
end

puts "render load: #{(AMY.render_load * 100).to_i}%"
