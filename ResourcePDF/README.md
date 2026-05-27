
# Robotics & Inverse Kinematics Reference Library

A curated collection of textbook chapters and research papers covering robot kinematics, trajectory planning, and numerical methods for inverse kinematics.

## Contents

### 📄 A General, Fast, and Robust Implementation of the Inverse Kinematics
A research paper presenting a practical IK solver design. Covers a general-purpose approach that balances computational speed with robustness to singularities and unreachable configurations. Useful as an implementation reference.

### 📄 Manipulator IK – Vectors, Formulas, and Damped Least Squares
A focused reference on the mathematics of IK for serial manipulators. Covers:
- Jacobian-based formulations
- Damped Least Squares (DLS / Levenberg–Marquardt) method for singularity handling
- Vector/matrix formulas ready for implementation

### 📄 Modern Robotics – Chapters 7–9 (Lynch & Park)
Chapters from the *Modern Robotics* textbook. Likely covers:
- Ch. 7: Kinematics of Closed Chains
- Ch. 8: Dynamics of Open Chains
- Ch. 9: Trajectory Generation
Pairs with the free online course at modernrobotics.org.

### 📄 Robotics: Modelling, Planning and Control – Sciavicco et al. (Extract)
An extract from the classic Sciavicco & Siciliano textbook. Covers DH parameters, workspace analysis, differential kinematics, and the Jacobian. A foundational reference for manipulator modeling.

## Topics Covered

- Forward & inverse kinematics
- Jacobian matrix and differential kinematics
- Damped Least Squares (DLS) for singularity robustness
- Trajectory planning
- Robot dynamics

## Recommended Reading Order

1. Sciavicco extract — foundational theory
2. MR Ch. 7–9 — modern geometric formulation
3. Manipulator IK vectors & DLS — math ready for code
4. General/Fast/Robust IK paper — implementation strategies
