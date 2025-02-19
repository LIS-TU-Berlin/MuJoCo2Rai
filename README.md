# Mujoco to RAI Converter  

A Python tool for converting MuJoCo XML files—including joints, meshes, and textures—into `.g` files with convex decomposition, fixed mass parameters, and vertex-wise colors, making them compatible with the RAI interface.  

## Features  
- Converts MuJoCo `.xml` files while preserving joints and mesh structures.  
- Applies convex decomposition and assigns fixed mass parameters.  
- Supports textures and vertex-wise coloring.  
- Outputs ready-to-use `.g` files for RAI.  

## Usage  

1. **Download the RoboCasa Dataset** (if working with RoboCasa fixtures).  
2. Place the `fixture` directory inside this repository.  
3. Run the conversion script:  

   ```bash
   python3 robocasa2rai.py
   ```  

This will generate jointed RAI `.g` files in the correct format.