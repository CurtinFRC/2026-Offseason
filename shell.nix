{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    pkgs.jdk21
    pkgs.python3
  ];

  shellHook = ''
    export JAVA_HOME=${pkgs.jdk21}
    echo "Java 21 development environment loaded"
    java -version
  '';
}
