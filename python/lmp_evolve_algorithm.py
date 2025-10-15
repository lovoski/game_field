import numpy as np
from typing import Callable

class Population:
    """
    An implementation of a genetic algorithm using NumPy for vectorized operations.
    
    This class manages a population of candidate solutions ('individuals') and evolves
    them over generations to find an optimal solution for a given fitness function.
    """

    def __init__(self,
                 size: int,
                 elites: int,
                 exploration: float,
                 lower_bounds: np.ndarray,
                 upper_bounds: np.ndarray,
                 seed: np.ndarray,
                 func: Callable[[np.ndarray], float]):
        
        # --- Basic Parameters ---
        self.size = size
        self.elites = elites
        self.exploration = exploration
        self.func = func
        
        self.lower_bounds = np.array(lower_bounds, dtype=np.float64)
        self.upper_bounds = np.array(upper_bounds, dtype=np.float64)
        self.dimensionality = len(seed)

        # Use NumPy's modern random number generator
        self.rng = np.random.default_rng()

        # --- Compute Rank Probabilities (Vectorized) ---
        ranks = np.arange(self.size, 0, -1)
        rank_sum = self.size * (self.size + 1) / 2.0
        self.rank_probabilities = ranks / rank_sum

        # --- Create Population Arrays (Struct of Arrays approach) ---
        # Current generation
        self.genes = np.zeros((self.size, self.dimensionality))
        self.momentum = np.zeros((self.size, self.dimensionality))
        self.fitness = np.full(self.size, np.inf)
        self.extinction = np.zeros(self.size)

        # Next generation (offspring)
        self.offspring_genes = np.zeros_like(self.genes)
        self.offspring_momentum = np.zeros_like(self.momentum)
        
        # --- Initialize Population ---
        self.genes[0] = np.array(seed, dtype=np.float64)
        # Randomly initialize the rest of the population
        self.genes[1:] = self.rng.uniform(
            self.lower_bounds,
            self.upper_bounds,
            size=(self.size - 1, self.dimensionality)
        )
        
        # --- Finalize Initialization ---
        self._evaluate_fitness(self.genes, self.fitness)
        self._sort_population()
        self._assign_extinctions()

    def get_solution(self) -> np.ndarray:
        """Returns the genes of the best individual in the population."""
        return self.genes[0]

    def evolve(self):
        """Performs one generation of evolution."""
        
        # --- 1. Elitism: Copy the best individuals directly to the next generation ---
        self.offspring_genes[:self.elites] = self.genes[:self.elites]
        self.offspring_momentum[:self.elites] = self.momentum[:self.elites]

        # --- 2. Crossover & Mutation for the rest of the population ---
        for o in range(self.elites, self.size):
            if self.rng.random() > self.exploration:
                # --- Selection ---
                # Select three unique parents based on rank probabilities
                parent_indices = self.rng.choice(self.size, size=3, replace=False, p=self.rank_probabilities)
                pA_idx, pB_idx, pP_idx = parent_indices
                
                parentA_genes, parentB_genes, prototype_genes = self.genes[pA_idx], self.genes[pB_idx], self.genes[pP_idx]
                parentA_momentum, parentB_momentum = self.momentum[pA_idx], self.momentum[pB_idx]
                parentA_ext, parentB_ext = self.extinction[pA_idx], self.extinction[pB_idx]
                
                # --- Crossover, Mutation, Adoption (Vectorized for all dimensions) ---
                extinction_avg = 0.5 * (parentA_ext + parentB_ext)
                mutation_rate = extinction_avg * (1.0 - 1.0 / self.dimensionality) + (1.0 / self.dimensionality)
                mutation_strength = extinction_avg

                # Store pre-mutation genes for momentum calculation
                child_genes = self.offspring_genes[o]
                
                # Recombination
                momentum = (self.rng.random(self.dimensionality) * parentA_momentum +
                            self.rng.random(self.dimensionality) * parentB_momentum)
                
                mask = self.rng.random(self.dimensionality) < 0.5
                child_genes[:] = np.where(mask, parentA_genes, parentB_genes) + momentum
                gene_snapshot = np.copy(child_genes)

                # Mutation
                if self.rng.random() < mutation_rate:
                    span = self.upper_bounds - self.lower_bounds
                    mutation_amount = self.rng.uniform(-mutation_strength * span, mutation_strength * span)
                    child_genes += mutation_amount
                
                # Adoption
                weight = self.rng.random()
                mid_parent_genes = 0.5 * (parentA_genes + parentB_genes)
                child_genes += (weight * self.rng.random() * (mid_parent_genes - child_genes) +
                                (1.0 - weight) * self.rng.random() * (prototype_genes - child_genes))
                
                # Clamp to bounds
                np.clip(child_genes, self.lower_bounds, self.upper_bounds, out=child_genes)
                
                # Momentum update
                self.offspring_momentum[o] = (self.rng.random(self.dimensionality) * momentum +
                                              (child_genes - gene_snapshot))
            else:
                # --- 3. Exploration: Create a new random individual ---
                self._reroll(self.offspring_genes[o], self.offspring_momentum[o])

        # --- 4. Finalize Generation ---
        # Create temporary fitness array for the new offspring
        offspring_fitness = np.full(self.size, np.inf)
        self._evaluate_fitness(self.offspring_genes, offspring_fitness)

        # Swap current generation with offspring
        self.genes, self.offspring_genes = self.offspring_genes, self.genes
        self.momentum, self.offspring_momentum = self.offspring_momentum, self.momentum
        self.fitness = offspring_fitness
        
        self._sort_population()
        self._assign_extinctions()
        
    def _reroll(self, genes: np.ndarray, momentum: np.ndarray):
        """Re-initializes an individual's genes and momentum."""
        genes[:] = self.rng.uniform(self.lower_bounds, self.upper_bounds)
        momentum[:] = 0.0

    def _evaluate_fitness(self, gene_pool: np.ndarray, fitness_pool: np.ndarray):
        """Evaluates and assigns fitness for a given pool of individuals."""
        for i in range(self.size):
            fitness_pool[i] = self.func(gene_pool[i])

    def _sort_population(self):
        """Sorts the entire population by fitness (lowest is best)."""
        sorted_indices = np.argsort(self.fitness)
        self.fitness = self.fitness[sorted_indices]
        self.genes = self.genes[sorted_indices]
        self.momentum = self.momentum[sorted_indices]
        self.extinction = self.extinction[sorted_indices] # Also needs sorting
        
    def _assign_extinctions(self):
        """Computes and assigns extinction values based on fitness ranking (Vectorized)."""
        min_fitness = self.fitness[0]
        max_fitness = self.fitness[-1]
        
        # Avoid division by zero if all fitness values are the same
        if max_fitness == 0:
            self.extinction.fill(0.0)
            return
            
        grading = np.arange(self.size) / (self.size - 1.0)
        self.extinction = (self.fitness + min_fitness * (grading - 1.0)) / max_fitness
        np.clip(self.extinction, 0.0, 1.0, out=self.extinction) # Ensure extinction is in [0, 1]