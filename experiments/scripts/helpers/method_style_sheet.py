import seaborn as sns
import regex as re
import numpy as np

SNS_BLUE = "#0173b2"
SNS_ORANGE = "#de8f05"
SNS_GREEN = "#029e73"
SNS_RED = "#d55e00"
SNS_PURPLE = "#cc78bc"
SNS_BROWN = "#ca9161"
SNS_PINK = "#fbafe4"
SNS_GREY = "#949494"
SNS_YELLOW = "#ece133"
SNS_LIGHT_BLUE = "#56b4e9"

METHOD_STYLE_SHEET = {
    "centralized": {
        "name": "Centralized (iSAM2)",
        "color": SNS_GREY,
        "symbol": "o",
        "linestyle": "solid",
    },
    "independent": {
        "name": "Independent",
        "color": SNS_PURPLE,
        "symbol": "d",
        "linestyle": "solid",
    },

    "ddfsam2": {
        "name": "DDF-SAM2",
        "color": SNS_GREEN,
        "symbol": "X",
        "linestyle": "solid",
    },
    "imesa": {
        "name": "iMESA",
        "color": SNS_BLUE,
        "symbol": "*",
        "linestyle": "solid",
    }
}
