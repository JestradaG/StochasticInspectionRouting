# Optimizing Inspection Routes and Schedules for Infrastructure Systems under Stochastic Decision-dependent Failures

The software and data in this repository are a snapshot of the software and data that were used in the research reported in the paper "Optimizing Inspection Routes and Schedules for Infrastructure Systems under Stochastic Decision-dependent Failures" by Juan-Alberto Estrada-Garcia and Siqian Shen.

# Description

This repository aims to show the efficiency of the scenario decomposition algorithm to solve stochastic two-stage inspection routing and scheduling problems. The objective is to minimize the total cost associated with fleet deployment, encompassing both scheduling and routing, balanced with the expected penalties incurred from undesirable failures. We model the problem as a variant of a stochastic multi-vehicle routing problem with decision-dependent endogenous uncertainty, and formulate a two-stage stochastic mixed-integer program based on finite samples. To efficiently solve the problem, we develop scenario decomposition, further enhanced by column generation and random coloring techniques to expedite the solution of decomposed subproblems. We conduct numerical studies on randomly generated networks of different sizes and topologies, as well as on IEEE 33-bus and IEEE 123-bus systems, representing power-grid inspection cases in practice.

## Data files
The `data/` directory contains the 33 and 123 bus IEEE instances from the paper "Schneider et al. (2018). Analytic considerations and design basis for the IEEE distribution test feeders"

The `helperFunctions/instanceGeneration.py` script reads and encodes IEEE and randomly generations instances to be solved.

## Code files
The `scenarioDecomposition/` package contains all the subproblems and implementations of our proposed algorithm. The `helperFunctions/` package includes functions that are called by all the methods. The `otherAlgorithms/` directory implements the heuristic we describe in the paper and the extended formulation we use to benchmark our proposed algorithm.

Repository for the extended results for the paper "Optimizing Inspection Routes and Schedules for Infrastructure Systems under Stochastic Decision-dependent Failures"

# Cite
To cite the contents of this repository, please cite both the paper and this repo, using their respective DOIs.
Below is the BibTex for citing this snapshot of the repository.

```
@misc{InspectionRoutingData,
  author =        {Juan-Alberto Estrada-Garcia and Siqian Shen},
  title =         {Optimizing Inspection Routes and Schedules for Infrastructure Systems under Stochastic Decision-dependent Failures},
  year =          {2026},
  note =          {Available for download at https://github.com/JestradaG/StochasticInspectionRouting},
}
 ```
