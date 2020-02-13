{ stdenv
, gcc-arm-embedded
, python3
, doxygen ? null
, enabelDoc ? true
, stlink
}:

assert enabelDoc -> doxygen != null;

stdenv.mkDerivation {
  name = "bluepill-pin-tester";
  buildInputs = [ gcc-arm-embedded python3 doxygen stlink ];
}
