# Level generator

Level generator written in rust for my upcoming AR game. It is able to randomly generate moving entities while making sure that none of them will intersect in a given volume of space.

## Problem
In 2D Bullet hell games enemies or level features are usually allowed to intersect. It won't look akward because one enemy covering the other may be explained by difference in position in non-existent 3rd axis. It won't work in 3D games however as intersecting meshes won't look really good and player expects that when two objects occupy the same space they will interact.

## Solution
We can randomly generate levels in such a way that none of the generated objects will ever intersect. When we generate a lot of entities this can get computationaly heavy so the solution is to check collisions between entities occupying some given bounded volume. This keeps the amount of collision checks required more or less constant during whole generation process.

## FFI
This lib comes with unsafe FFI as it is intended to be used with Unity where it can be called with DllImport
