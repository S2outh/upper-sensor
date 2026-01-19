{
  description = "embassy h723 flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, fenix, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ fenix.overlays.default ];
        };
      in
      {
        devShells.default =
        let
          toolchain = pkgs.fenix.toolchainOf {
            channel = "nightly";
            date = "2025-12-22";
            sha256 = "sha256-bXz1imrwFz4Z5vlZV4jfRZWwsRma6Sk95IOuTMQFFVU=";
          };
          lib = pkgs.fenix.targets.thumbv7em-none-eabihf.toolchainOf {
            channel = "nightly";
            date = "2025-12-22";
            sha256 = "sha256-bXz1imrwFz4Z5vlZV4jfRZWwsRma6Sk95IOuTMQFFVU=";
          };
          rust = pkgs.fenix.combine [
            toolchain.rustc
            toolchain.rust-src
            toolchain.cargo
            toolchain.rustfmt
            toolchain.clippy
            lib.rust-std
          ];
        in
        pkgs.mkShell {
          buildInputs = with pkgs; [
            rust
            cargo-edit

            # for flashing
            probe-rs-tools

            # for external deps
            pkg-config
          ];

					# set default defmt log level
					DEFMT_LOG = "info";
        };
      }
    );
}
