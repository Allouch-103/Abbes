# Prompt pour IA de présentation (Gamma / Tome / Beautiful.ai)

> Copier-coller le bloc ci-dessous. Remplir les `[IMAGE: ...]` avec les figures
> de `report/figures/`.

---

Crée une présentation de soutenance de PFE (INSAT / Université de Carthage,
3ᵉ année Génie Logiciel, 2025/2026).

**Sujet :** « Abbes », robot humanoïde 18 DDL conçu de A à Z.
**Auteurs :** Zeyneb Benabdallah & Yassine Allouch. **Encadrante :** Mme Hajer Taktak.

**Style (strict) :**
- Phrases très courtes, puces, 1 idée par diapo. Jamais de paragraphes.
- Chaque concept expliqué en 1 ligne (ZMP, ROS2, Ollama...).
- Un emplacement image par diapo technique : `[IMAGE: ...]`.
- Insister sur la partie logicielle : ROS2, simulation, Ollama.

**Structure (9 diapos) :**

1. **Page de garde** — titre, auteurs, encadrante, établissement, logos.
   `[IMAGE: logo_insat.png + logo_carthage.png]`

2. **Plan** — vue d'ensemble numérotée des 6 sections.

3. **01 · Contexte & Problématique** — pourquoi ce projet, défis (équilibre,
   marche bipède, commande haut niveau). `[IMAGE: fig_real_robot.png]`

4. **02 · État de l'art** — robots existants + comparatif, travail antérieur
   (Selmi 2024). `[IMAGE: robot_asimo.png / robot_hrp2.png]`

5. **03 · Architecture & Spécifications** — couches logicielles + hardware :
   Web UI → LLM (Ollama) → API → ROS2 → (Gazebo | ESP32). `[IMAGE: arch_summary.pdf]`

6. **04 · Conception & Implémentation** — ROS2 (nœuds/topics, ros2_control),
   marche ZMP/LIPM, IK, PID. `[IMAGE: arch_node_graph.pdf + fig_ik_sagittal.png]`

7. **05 · Résultats** — phases réalisées + métriques (sim Gazebo, marche sur
   place, couche IA Ollama). `[IMAGE: arch_gz_ros2control.pdf]`

8. **06 · Conclusion & Perspectives** — bilan + à venir (RL pour la marche,
   parité Gazebo sim→réel).

9. **Clôture** — Merci / Questions. `[IMAGE: fig_real_robot.png]`
