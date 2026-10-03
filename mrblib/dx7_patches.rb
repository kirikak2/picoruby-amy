# Names of AMY's built-in DX7 presets (patches 128..255), for UI labels.
#
# AMY keeps the names only as comments in its patch table, so this list is
# generated from lib/amy/src/patches.h ("/* 130: DX7 BRASS   3  */" ->
# "BRASS 3"). Regenerate it when the AMY submodule is updated.
module AMY
  DX7_FIRST_PATCH = 128

  DX7_PATCH_NAMES = [
    "BRASS 1", "BRASS 2", "BRASS 3", "STRINGS 1", "STRINGS 2", "STRINGS 3",
    "ORCHESTRA", "PIANO 1", "PIANO 2", "PIANO 3", "E.PIANO 1", "GUITAR 1",
    "GUITAR 2", "SYN-LEAD 1", "BASS 1", "BASS 2", "E.ORGAN 1", "PIPES 1",
    "HARPSICH 1", "CLAV 1", "VIBE 1", "MARIMBA", "KOTO", "FLUTE 1",
    "ORCH-CHIME", "TUB BELLS", "STEEL DRUM", "TIMPANI", "REFS WHISL",
    "VOICE 1", "TRAIN", "TAKE OFF", "PIANO 4", "PIANO 5", "E.PIANO 2",
    "E.PIANO 3", "E.PIANO 4", "PIANO 5THS", "CELESTE", "TOY PIANO",
    "HARPSICH 2", "HARPSICH 3", "CLAV 2", "CLAV 3", "E.ORGAN 2", "E.ORGAN 3",
    "E.ORGAN 4", "E.ORGAN 5", "PIPES 2", "PIPES 3", "PIPES 4", "CALIOPE",
    "ACCORDION", "SITAR", "GUITAR 3", "GUITAR 4", "GUITAR 5", "GUITAR 6",
    "LUTE", "BANJO", "HARP 1", "HARP 2", "BASS 3", "BASS 4", "PICCOLO",
    "FLUTE 2", "OBOE", "CLARINET", "SAX BC", "BASSOON", "STRINGS 4",
    "STRINGS 5", "STRINGS 6", "STRINGS 7", "STRINGS 8", "BRASS 4", "BRASS 5",
    "BRASS 6 BC", "BRASS 7", "BRASS 8", "RECORDER", "HARMONICA1",
    "HRMNCA2 BC", "VOICE 2", "VOICE 3", "GLOKENSPL", "VIBE 2", "XYLOPHONE",
    "CHIMES", "GONG 1", "GONG 2", "BELLS", "COW BELL", "BLOCK", "FLEXATONE",
    "LOG DRUM", "SYN-LEAD 2", "SYN-LEAD 3", "SYN-LEAD 4", "SYN-LEAD 5",
    "SYN-CLAV 1", "SYN-CLAV 2", "SYN-CLAV 3", "SYN-PIANO", "SYNBRASS 1",
    "SYNBRASS 2", "SYNORGAN 1", "SYNORGAN 2", "SYN-VOX", "SYN-ORCH",
    "SYN-BASS 1", "SYN-BASS 2", "HARP-FLUTE", "BELL-FLUTE", "E.P-BRS BC",
    "T.BL-EXPA", "CHIME-STRG", "B.DRM-SNAR", "SHIMMER", "EVOLUTION",
    "WATER GDN", "WASP STING", "LASER GUN", "DESCENT", "OCTAVE WAR",
    "GRAND PRIX", "ST.HELENS", "EXPLOSION"
  ]

  # Display name of an AMY patch number: the DX7 name for 128..255, else the
  # number itself ("Juno 5", "Patch 1024").
  def self.patch_name(number)
    i = number - DX7_FIRST_PATCH
    return DX7_PATCH_NAMES[i] if i >= 0 && i < DX7_PATCH_NAMES.size
    return "Juno #{number}" if number >= 0 && number < DX7_FIRST_PATCH
    "Patch #{number}"
  end
end
