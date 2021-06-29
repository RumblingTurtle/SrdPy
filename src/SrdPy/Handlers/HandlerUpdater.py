class HandlerUpdater():
    
    def __init__(self):
        self.handlers = []

    def update(self):
        for handler in self.handlers:
            handler.update()