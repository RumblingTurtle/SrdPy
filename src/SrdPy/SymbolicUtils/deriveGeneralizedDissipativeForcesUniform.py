from SrdPy import SymbolicEngine


def deriveGeneralizedDissipativeForcesUniform(symbolicEngine:SymbolicEngine, uniformCoefficient):
    return -uniformCoefficient * symbolicEngine.v