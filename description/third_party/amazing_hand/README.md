# AmazingHand meshes

`left_hand.stl` and `right_hand.stl` are derived from the **AmazingHand** by **Pollen Robotics**
(https://github.com/pollen-robotics/AmazingHand), © Pollen Robotics. They are licensed under the
[Creative Commons Attribution 4.0 International licence (CC-BY-4.0)](https://creativecommons.org/licenses/by/4.0/),
like the original mechanical design; they are not covered by this repository's own licences.

Changes from the original: the parts of each hand were placed at their assembled pose (from the
project's MuJoCo models, with the palm and top shells from its `cad/stl` files), the fasteners,
pins and bushings left out, each part simplified, the parts united into one closed solid,
decimated to 30,000 triangles and expressed in metres in the hand's frame (wrist interface face
at z = 0, fingers along +z, palm towards +x). `tools/build_hand_mesh.py` rebuilds them from a
clone of the original repository.

Marvin Bot uses these meshes to show the hands in its CAD model, renders and URDF, and to check
the hands' clearance through the joint ranges. The hands themselves are bought as kits. The
names "AmazingHand" and "Pollen Robotics" are used here only to attribute the design; this
project is not affiliated with or endorsed by Pollen Robotics.
