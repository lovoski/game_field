import numpy as np
from lmp_evolve_algorithm import Population

def sphere(x):
    """Convex, unimodal — optimum = 0 at x=0"""
    return np.sum(x**2)

def rastrigin(x):
    """Highly multimodal — optimum = 0 at x=0"""
    A = 10
    return A * len(x) + np.sum(x**2 - A * np.cos(2 * np.pi * x))

def rosenbrock(x):
    """Non-convex, narrow valley — optimum = 0 at x=[1,1,...,1]"""
    return np.sum(100.0 * (x[1:] - x[:-1]**2)**2 + (1 - x[:-1])**2)

def ackley(x):
    """Multimodal — optimum = 0 at x=0"""
    a, b, c = 20, 0.2, 2*np.pi
    n = len(x)
    sum_sq = np.sum(x**2)
    sum_cos = np.sum(np.cos(c * x))
    return -a * np.exp(-b * np.sqrt(sum_sq / n)) - np.exp(sum_cos / n) + a + np.e

def schwefel(x):
    """Many local minima — optimum = 0 at x=420.9687..."""
    n = len(x)
    return 418.9829 * n - np.sum(x * np.sin(np.sqrt(np.abs(x))))

def griewank(x):
    """Many local minima — optimum = 0 at x=0"""
    i = np.arange(1, len(x) + 1)
    return np.sum(x**2) / 4000 - np.prod(np.cos(x / np.sqrt(i))) + 1

# --- Example Usage ---
if __name__ == '__main__':
    # --- Parameters ---
    POPULATION_SIZE = 50
    ELITES = 5
    EXPLORATION = 0.2
    DIMENSIONALITY = 5
    LOWER_BOUNDS = np.array([-10.0] * DIMENSIONALITY)
    UPPER_BOUNDS = np.array([10.0] * DIMENSIONALITY)
    INITIAL_SEED = np.array([5.0, -8.0, 2.0, 13.0, 2.0])
    MAX_GENERATIONS = 400

    # --- Initialization ---
    pop = Population(
        size=POPULATION_SIZE,
        elites=ELITES,
        exploration=EXPLORATION,
        lower_bounds=LOWER_BOUNDS,
        upper_bounds=UPPER_BOUNDS,
        seed=INITIAL_SEED,
        func=sphere
    )

    # --- Evolution Loop ---
    print(f"Starting evolution...")
    print(f"Generation 0 | Best Fitness: {pop.fitness[0]:.6f} | Solution: {np.round(pop.get_solution(), 3)}")

    for gen in range(1, MAX_GENERATIONS + 1):
        pop.evolve()
        if gen % 10 == 0:
            print(f"Generation {gen} | Best Fitness: {pop.fitness[0]:.6f} | Solution: {np.round(pop.get_solution(), 3)}")

    print("\nEvolution finished!")
    print(f"Final Best Fitness: {pop.fitness[0]:.6f}")
    print(f"Final Solution: {pop.get_solution()}")