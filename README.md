# stewart_control ROS2 Project

Ce dépôt contient un workspace ROS2 (`colcon`) dédié au contrôle d'une plateforme Stewart.

## Structure

- `src/stewart_control` : paquet principal contenant les noeuds ROS et le code de cinématique.
- `src/stewart_control/Arduino` : code embarqué pour la carte contrôlant les vérins.

## Compilation

```bash
# depuis la racine du workspace
eval "$(ros2 env)"          # source l'environnement ROS2
colcon build --symlink-install
```

## Exécution

```bash
# lancer les noeuds de capteurs et fusion
ros2 launch stewart_control stewart_ordered_launch.py

# ensuite démarrer stewart_node ou manual_stewart_node
ros2 run stewart_control stewart_node
```

## Branches Git

- `main` : version stable
- `develop` : intégration des fonctionnalités en cours
- branches de fonctionnalités pour chaque ticket

## Qualité du code

Ce projet utilise les outils suivants :

* **Black** pour le formatage Python
* **flake8** pour le linting Python
* **clang-format** pour la mise en forme du code Arduino/C++
* **pre-commit** pour lancer automatiquement ces vérifications avant chaque commit

Après avoir cloné le dépôt, installez les hooks :

```bash
pip install --user pre-commit
pre-commit install
# lancer manuellement sur tous les fichiers pour la première fois
pre-commit run --all-files
```

## Contributions

1. Créez une nouvelle branche `feature/...` ou `bugfix/...`.
2. Formatez le code (`black`, `clang-format` pour Arduino).
3. Ajoutez/modifiez des tests si nécessaire.
4. Faites une pull request.

---

*Documentation basique créée le 26 février 2026.*
