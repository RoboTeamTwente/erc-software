{ inputs, cell }:
let
  inherit (inputs) nixpkgs;
  inherit (inputs.std.lib.dev) mkShell;
  inherit (inputs.std) std;

  l = builtins // nixpkgs.lib;
in
nixpkgs.lib.mapAttrs (_: mkShell) {
  default = {
    name = "ERC";
    imports = [
      std.devshellProfiles.default
    ];
    packages = [
      nixpkgs.clang-tools
      nixpkgs.clang
      nixpkgs.skopeo
    ];
    commands = [
      { package = nixpkgs.cocogitto; }
      { package = nixpkgs.gh; }
      { package = nixpkgs.just; }
    ];
    nixago = [
      cell.dotfiles.lefthook
      cell.dotfiles.treefmt
    ];
  };
}
