{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    pkgs.cmake
    pkgs.ninja
    pkgs.pkg-config
    pkgs.pkgsCross.armv7l-hf-multiplatform.pkgsMusl.stdenv.cc   # ssc338q gcc/g++
  ];
}
