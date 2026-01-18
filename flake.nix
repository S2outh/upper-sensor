{
  description = "Rust/Embassy H723 Flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix/monthly";
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

        rust-target = pkgs.fenix.targets.thumbv7em-none-eabihf.latest;

        rust-toolchain = pkgs.fenix.combine [
          (pkgs.fenix.latest.withComponents [
            "cargo"
            "rustc"
            "rust-src"
            "rustfmt"
            "clippy"
          ])

          rust-target.rust-std
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            rust-toolchain
            probe-rs-tools  # flashing tools

            # for external deps
            pkg-config
          ];

          # Environment variables for rust-analyzer and logging
          RUST_SRC_PATH = "${rust-toolchain}/lib/rustlib/src/rust/library";
          DEFMT_LOG = "info";
        };
      }
    );
}
