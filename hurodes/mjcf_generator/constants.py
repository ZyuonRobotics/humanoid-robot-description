# texture
DEFAULT_GROUND_TEXTURE_ATTR = {
    "type": "2d",
    "name": "groundplane",
    "builtin": "checker",
    "mark": "edge",
    "rgb1": "0.2 0.3 0.4",
    "rgb2": "0.1 0.2 0.3",
    "markrgb": "0.8 0.8 0.8",
    "width": "300",
    "height": "300"
}
DEFAULT_SKY_TEXTURE_ATTR = {
    "type": "skybox",
    "builtin": "gradient",
    "rgb1": "0.3 0.5 0.7",
    "rgb2": "0 0 0",
    "width": "512",
    "height": "3072"
}

# material
DEFAULT_GROUND_MATERIAL_ATTR ={
    "name": "groundplane",
    "texture": "groundplane",
    "texuniform": "true",
    "texrepeat": "5 5",
    "reflectance": "0.2"
}

# geom
DEFAULT_GROUND_GEOM_ATTR = {
    "name": "floor",
    "size": "0 0 0.05",
    "type": "plane",
    "material": "groundplane",
    "condim": "3",
    "conaffinity": "15"
}

# light
DEFAULT_SKY_LIGHT_ATTR =  {
    "pos": "0 0 3.5",
    "dir": "0 0 -1",
    "directional": "true"
}