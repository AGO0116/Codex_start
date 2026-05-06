# PLATEAU 3D Tiles Placement

Place `tileset.json` and its matching `data/` directory under this folder without changing their relative paths.

Example:

```text
web/data/plateau/shibuya/
|- tileset.json
`- data/
   |- 0.b3dm
   |- 1.b3dm
   `- ...
```

In the Viewer, open `3D Tiles` mode and load:

```text
./data/plateau/shibuya/tileset.json
```

Notes:

- Do not separate `tileset.json` from its `data/` folder.
- Serve the project over HTTP, for example with `python -m http.server 8000`.
- The current 3D Tiles mode is an initial integration for city rendering and robot movement.
- Automatic walkable-area or obstacle extraction from PLATEAU geometry is not implemented yet.
