import json

class Settings:
    def __init__(self, path:str):
        self.path = path
        try:
            with open(path, 'r') as fl:
                self.data = json.load(fl)
        except FileNotFoundError:
            self.data = {}
    
    def __getitem__(self, name):
        return self.data[name]
    
    def __setitem__(self, name, value):
        self.data[name] = value
    
    def save(self):
        with open(self.path, 'w') as fl:
            json.dump(self.data, fl, indent=2)