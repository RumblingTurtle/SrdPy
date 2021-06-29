from itertools import product
from multiprocessing import Pool


def callForAllCombinations(inputOne, inputTwo, functionReference):
    inputTuples = product(inputOne, inputTwo)
    if Math.useParallel:
        with Pool(Math.numberOfWorkers) as pool:
            outputs = pool.starmap(functionReference, iterable=inputTuples)
    else:
        outputs = []
        for inputTuple in inputTuples:
            outputs.append(functionReference(inputTuple[0], inputTuple[1]))

    xInputs, yInputs = zip(*product(inputOne, inputTwo))
    return list(xInputs), list(yInputs), list(outputs)