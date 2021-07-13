class HandlerUpdater():
    
    def __init__(self,handlers=[]):
        self.handlers = handlers

    def update(self):
        for handler in self.handlers:
            handler.update()