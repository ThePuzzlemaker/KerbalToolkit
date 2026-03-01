{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    systems.url = "github:nix-systems/default";
    systems.flake = false;
  };

  outputs = {
    nixpkgs,
    systems,
    ...
  }: let
    eachSystem = nixpkgs.lib.genAttrs (import systems);
  in {
    formatter = eachSystem (system: nixpkgs.legacyPackages.${system}.alejandra);

    devShells = eachSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};
      qtbase = pkgs.qt6.qtbase;
      qtwayland = pkgs.qt6.qtwayland;
    in {
      default = pkgs.mkShell#.override { stdenv = pkgs.clangStdenv; }
       rec {
        packages = with pkgs; [
	        clang-tools # https://github.com/NixOS/nixpkgs/issues/214945
            cmake
            ninja
            eigen
            pagmo2
            boost
            nlopt
            ipopt
            openblas
            qt6.qtbase
            qt6.qtwayland
            qt6.qtsvg
        ];

        LD_LIBRARY_PATH = nixpkgs.lib.makeLibraryPath packages;
        QT_PLUGIN_PATH = "${qtbase}/${qtbase.qtPluginPrefix}:${pkgs.qt6.qtwayland}/${qtbase.qtPluginPrefix}";
        QT6_QML_IMPORT_PATH = "${qtbase}/${qtbase.qtQmlPrefix}:${pkgs.qt6.qtwayland}/${qtbase.qtQmlPrefix}";
      };
    });
  };
}
