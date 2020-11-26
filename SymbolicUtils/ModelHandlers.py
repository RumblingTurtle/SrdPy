class SRDModelHandler:
    def __init__(self):
        model_type_code = 1
        #options: 0 - 'not configured', 
        #         1 - 'exact'
        #         2 - 'OrtegaSpong' - with parameters as inputs,
        #         regressor-style
        #         3 - 'numeric'
        
        ###########
        #function handles
        
        
        self.mechanicalEquationsNumeric = []
        
        ###########
        #state
        
        theta = [] #Ortega-Spong model parameters - actual value
        estimatedTheta = [] #Ortega-Spong model parameters - estimate
        
        numericFunctionsUpdatedTime = 0
        numericFunctionsValue = []
