{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    pkgs.cmake
    pkgs.ninja
    pkgs.pkg-config
  ];
}
