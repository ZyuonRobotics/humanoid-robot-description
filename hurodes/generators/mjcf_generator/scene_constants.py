PLANE_NAME = "plane"

# visual
DEFAULT_VISUAL_ELEM = {
    "headlight": {
        "diffuse": "0.6 0.6 0.6",
        "ambient": "0.3 0.3 0.3",
        "specular": "0 0 0"
    },
    "rgba": {
        "haze": "0.15 0.25 0.35 1"
    },
    "global": {
        "azimuth": "160",
        "elevation": "-20"
    }
}

# texture
DEFAULT_TEXTURE_ELEM = {
    "plane": {
        "type": "2d",
        "name": PLANE_NAME,
        "builtin": "checker",
        "mark": "edge",
        "rgb1": "0.2 0.3 0.4",
        "rgb2": "0.1 0.2 0.3",
        "markrgb": "0.8 0.8 0.8",
        "width": "300",
        "height": "300"
    },
    "skybox": {
        "type": "skybox",
        "builtin": "gradient",
        "rgb1": "0.3 0.5 0.7",
        "rgb2": "0 0 0",
        "width": "512",
        "height": "3072"
    }
}

# material
DEFAULT_MATERIAL_ATTR ={
    "plane": {
        "name": PLANE_NAME,
        "texture": PLANE_NAME,
        "texuniform": "true",
        "texrepeat": "5 5",
        "reflectance": "0.2"
    }
}

# light
DEFAULT_LIGHT_ATTR =  {
    "pos": "0 0 3.5",
    "dir": "0 0 -1",
    "directional": "true"
}


# geom
DEFAULT_GROUND_GEOM_ATTR = {
    "name": "floor",
    "size": "0 0 0.05",
    "type": "plane",
    "material": PLANE_NAME,
    "condim": "1",
    "conaffinity": "1"
}
