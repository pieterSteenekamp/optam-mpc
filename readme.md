### Nonlinear MPC / APC Proof of Concept

This repository contains a proof-of-concept implementation exploring the combination of:

- process engineering,

- first-principles and brown-box modelling,

- open-source optimisation libraries, and

- AI-assisted software development.

Many years ago I developed a commercial Model Predictive Control (MPC) software package from first principles and applied it successfully to a number of industrial processes. Returning to APC after several years of work in artificial intelligence and machine learning, I became interested in a different question:

What becomes possible today when modern computing power, open-source optimisation tools, and AI-assisted software development are combined?

Classical industrial MPC is dominated by linear approaches, typically based on step-response (DMC-style) models or state-space formulations. These methods have been enormously successful, and this project is not intended to diminish their value. In many industries, especially refining and petrochemicals, MPC remains a major competitive advantage.

The objective of this repository is to explore whether a more naturally nonlinear, first-principles-oriented approach can be made practical using contemporary open-source tools.

### Current Demonstration Models

- Four-tank circulating system inspired by aspects of chemical recovery circuits.

- Simplified lime-kiln model capturing nonlinear interactions between temperature, residence time, feed rate, and reaction dynamics.

### Important Disclaimer

This project is an experimental prototype and is not a mature industrial MPC product. Significant technical challenges remain, including:

- model quality,

- robustness,

- computational performance,

- constraint handling, and

- industrial deployment.

AI tools have not replaced engineering knowledge in this work; they have been used to accelerate experimentation, prototyping, and feature development.

Contributions, discussions, and technical feedback are welcome.

Author: Pieter Steenekamp Contact: pieters@randcontrols.co.za
