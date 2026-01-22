# Craft Project

The aim of this project is provide such a voxel game that is:

1. Natively multi-player supported, the client side make a change to the world, the server side verify and broadcast it to other visible players.
2. Support a really large chunk visibility distance both in terms of networking and rendering, this should be efficient. Maybe through dynamic chunk LODs, GPU based pipeline and chunk versioning.
3. The base game should support extension to different game logic, make it easy to create mini-games (Rule-based parktour, PvP, RPG games, puzzle games etc.).
