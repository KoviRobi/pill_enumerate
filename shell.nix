{}@args:
let pkgs = import <nixpkgs> {};
in pkgs.callPackage ./. args
