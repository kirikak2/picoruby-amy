MRuby::Gem::Specification.new('picoruby-amy') do |spec|
  spec.license = 'MIT'
  spec.author  = 'Toshio Maki'
  spec.summary = 'AMY synthesizer (FM / analog / PCM) as a MIDI transport for PicoRuby'
  spec.require_name = 'amy'
  spec.add_dependency 'picoruby-machine'
end
