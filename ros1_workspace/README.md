Ce dossier contient tout le necessaire au fonctionnement d'un projet ros1, dans lequel un masque est applique sur une image equirectangulaire pour cacher le robot.
Il contient:
- Un package pour faire tourner une camera kodak, qui publie une image equirectangulaire sur le topic `/kodak/equi`,
- Un package pour simuler le fait que les articulations du robot sont en mouvement, dans le cas ou on n'a pas le robot sous la main, et qui publie les angles des articulations sur le topic `/robot/joint_states`,
- Un package pour calibrer la camera virtuelle, en la positionnant et orientant dans la simulation, sur fond de l'image kodak, et qui une fois la calibration satisfaisante, reecrit le fichier de configuration du robot simulé.
- Un package pour estimer en temps reel la position du robot dans l'image equirectangulaire, et qui publie un masque representant le robot sur le topic `/mask_equirect`.

Pour l'instant le fait de lancer la visualisation exterieure du robot en simulation fait tout planter, il faudra palier ce probleme a l'avenir pour verifier que le jumeau numerique ne parte pas aux fraises.