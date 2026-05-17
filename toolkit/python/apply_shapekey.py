"""Apply one or more blend shapes (shape keys) to the Basis.

Each selected key is absorbed into Basis at its current value.
Every surviving shape key is shifted by the same cumulative delta so its
visual offset from the new Basis is unchanged.  Applied keys are removed.

Usage
-----
Paste into the Blender Python console, or run headless:
    blender --background scene.blend --python apply_shapekey.py

API
---
apply_shapekeys_to_basis()
    Apply only the currently active shape key.

apply_shapekeys_to_basis(names=["Blink_L", "Blink_R"])
    Apply the named shape keys.

apply_shapekeys_to_basis(names=None)          # same as no argument
apply_shapekeys_to_basis(names=["Smile"])     # single key by name
"""

import bpy
from mathutils import Vector


def apply_shapekeys_to_basis(obj=None, names=None):
    """
    Parameters
    ----------
    obj   : bpy.types.Object or None
        Target mesh object.  Defaults to the active object.
    names : list[str] or None
        Names of shape keys to bake into Basis.
        Pass None (default) to use only the active shape key.
    """
    if obj is None:
        obj = bpy.context.active_object
    if obj is None or obj.type != "MESH":
        raise RuntimeError("Active object must be a mesh.")

    keys = obj.data.shape_keys
    if keys is None or len(keys.key_blocks) < 2:
        raise RuntimeError("Object has no shape keys.")

    key_blocks = keys.key_blocks
    basis      = key_blocks[0]
    n          = len(basis.data)

    # ── Resolve the set of keys to apply ──────────────────────────────────
    if names is None:
        active_idx = obj.active_shape_key_index
        if active_idx == 0:
            raise RuntimeError("Select a shape key other than Basis.")
        to_apply = [key_blocks[active_idx]]
    else:
        to_apply = []
        for name in names:
            key = key_blocks.get(name)
            if key is None:
                raise RuntimeError(f"Shape key '{name}' not found.")
            if key is basis:
                raise RuntimeError("Cannot apply the Basis to itself.")
            to_apply.append(key)

    to_apply_set = set(to_apply)

    # ── Accumulate total delta across all keys being applied ───────────────
    # Shape keys are additive: each key's delta is independent of the others,
    # so the combined displacement is simply the sum of each key's contribution
    # at its current value.
    total_delta = [Vector((0.0, 0.0, 0.0))] * n
    for key in to_apply:
        v = key.value
        total_delta = [
            total_delta[i] + v * (key.data[i].co - basis.data[i].co)
            for i in range(n)
        ]

    # ── Shift Basis and every surviving sibling by the total delta ─────────
    for key in key_blocks:
        if key in to_apply_set:
            continue
        for i in range(n):
            key.data[i].co += total_delta[i]

    # ── Remove applied keys (iterate by name; indices shift on removal) ────
    applied_names = [k.name for k in to_apply]
    for name in applied_names:
        idx = key_blocks.find(name)
        if idx == -1:
            continue
        obj.active_shape_key_index = idx
        bpy.ops.object.shape_key_remove()

    print(f"[apply_shapekey] baked into Basis: {applied_names}")


# ── Default: apply the active shape key ───────────────────────────────────────
apply_shapekeys_to_basis()

# ── Examples (uncomment to use) ───────────────────────────────────────────────
# apply_shapekeys_to_basis(names=["Blink_L", "Blink_R"])
# apply_shapekeys_to_basis(names=["Smile"])
