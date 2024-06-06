# Robot Mask Simulation
Cette bibliotheque est un exemple d'integration de camera 360 et de projection equirectangulaire dans MuJoCo. Elle s'integre dans un projet aui vise a estimer, dans une image equirectangulaire, les pixels correspondant au robot sur base d'un jumeau numerique, afin de pouvoir masquer ces pixels lors de caluls de commandes.


### Camera 360 et projection Equirectangulaire
La plupart des moteurs de rendu graphique n'integrent pas par defaut de cameras 360 ou de camera fisheye. Ici, au lieu de venir modifier le rendu, on propose une surcouche simple a implementer dans n'importe quelle simulation.

Il s'agit simplement d'attacher, la ou on souhaite mettre la camera, *6 cameras perspectives*, toutes de *meme centre optique*, de *fov 90 °*, et orientees de maniere a former un cube.

Ensuite, sur la base du travail de Iacopo Catalano (https://github.com/IacopomC/Equirectangular-Cubemaps-Transform-CUDA-OpenMp), on transforme ces 6 images en une image equirectangulaire.

# File Tree
L'arborescence du projet est la suivante:
```
📦
 ┣ 📂 assets
 ┃ ┣ 📂 assets_ur10
 ┃ ┣ 📂 assets_ur5
 ┃ ┣ 📜 bzh.png
 ┃ ┣ 📜 mug.obj
 ┃ ┣ 📜 scene.xml
 ┃ ┣ 📜 scene_color.xml
 ┃ ┣ 📜 ur10e.xml
 ┃ ┗ 📜 ur5e.xml
 ┃
 ┣ 📂 examples
 ┃ ┣ 📂 ros1_package / equirect_mask_package
 ┃ ┣ 🐍 cubemap_to_equirect.py
 ┃ ┗ 🐍 keyboard_UR5_control.py
 ┃
 ┣ 📂 lib
 ┃ ┣ 🐍 MyRobot.py
 ┃ ┗ 🐍 Tiles2EquiCU.py
 ┃
 ┣ 📂 utils
 ┃ ┣ 📂 ros1_joint_state_minimal_publisher_package
 ┃ ┗ 🐍 cubemap_vizualisation.py
 ┃
 ┗ 📜 requirements.txt
```

-   `assets/` contient les fichiers de scene et les assets necessaires pour la simulation. Vous y trouverez de quoi simuler un ur5 ou un ur10, ainsi qu'un mug et une texture de drapeau breton. Sont proposes aussi des fichiers `scene.xml`, l'une en couleur et l'autre en noir et blanc et sans texture, reflet... etc.

-   `lib/` contient la classe `MyRobot` qui permet de charger un robot dans MuJoCo et de recuperer les images des cameras. `Tiles2EquiCU.py` contient les fonctions permettant de transformer les 6 images en une image equirectangulaire.

-   `utils/` contient differents scripts pour le debuggage, la disposition des images en cubemap, et plus tard peut-etre des scripts de calibration.

### Note sur les assets
`scene_color.xml` est une scene complete avec un ur5, un mug, un sol, un drapeau breton en texture, et un eclairage. Elle est principalement utilisee pour identifier les correspondances entre cameras et faces de la cubemap. Ca peut etre remplace par n'importe quelle scene, tant qu'elle contient un robot et un ou plusieurs objets visibles par les cameras.
