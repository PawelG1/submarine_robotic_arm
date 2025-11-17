import json
from .servo_model import Servo

class WorkCycle:
    
    def __init__(self):
        self.history = []
    
    def addStep(self, servos: list[Servo]):
        point = {
            "step_id": len(self.history),
            "type": "servo",
            "servos": {}
        }
        for servo in servos:
            point["servos"][f"servo_{servo.id}"] = servo.value
        self.history.append(point)
    
    def addDelayStep(self, delay_ms: int):
        """Dodaje krok typu delay z opóźnieniem w milisekundach"""
        point = {
            "step_id": len(self.history),
            "type": "delay",
            "delay_ms": delay_ms
        }
        self.history.append(point)
    
    def removeStep(self, step_id: int):
        """Usuwa krok o podanym step_id i reindeksuje pozostałe"""
        if 0 <= step_id < len(self.history):
            self.history.pop(step_id)
            # Re-index step_ids
            for index, step in enumerate(self.history):
                step["step_id"] = index

    def getHistory(self):
        return self.history
    
    def saveToFile(self, filename: str):
        import json
        with open(filename, 'w') as f:
            json.dump(self.history, f, indent=4)
    
    def loadFromFile(self, filename: str):
        try:
            with open(filename, 'r') as f:
                self.history = json.load(f)
        except FileNotFoundError:
            print(f"File {filename} not found. Starting with empty history.")
            self.history = []
        except json.JSONDecodeError:
            print(f"Error decoding JSON from {filename}. Starting with empty history.")
            self.history = []

    def clearHistory(self):
        self.history = []
    
    async def executeOnce(self, executor_callback):
        """Wykonuje jeden cykl work cycle, przekazując każdy krok do callback'a
        
        Args:
            executor_callback: async function(step) która obsługuje wykonanie kroku
        """
        import asyncio
        for step in self.history:
            if step["type"] == "servo":
                await executor_callback(step)
            elif step["type"] == "delay":
                delay_sec = step["delay_ms"] / 1000.0
                await asyncio.sleep(delay_sec)
    
    def setCyclicMode(self, enabled: bool):
        """Włącza/wyłącza tryb cykliczny"""
        self.cyclic_enabled = enabled
    
    def isCyclicEnabled(self) -> bool:
        """Sprawdza czy tryb cykliczny jest włączony"""
        return getattr(self, 'cyclic_enabled', False)
    
    async def executeCyclic(self, executor_callback):
        """Wykonuje work cycle w pętli, dopóki cyclic_enabled == True
        
        Args:
            executor_callback: async function(step) która obsługuje wykonanie kroku
        """
        while self.isCyclicEnabled():
            await self.executeOnce(executor_callback)

if __name__ == '__main__':
    cycle = WorkCycle()
    servos = [Servo(0, 90), Servo(1, 45), Servo(2, 135)]
    cycle.addStep(servos)
    servos[0].setValue(100)
    servos[1].setValue(60)
    cycle.addStep(servos)
    cycle.saveToFile('work_cycle.json')
    print(cycle.getHistory())
    cycle.clearHistory()
    print("cleared hist: ",cycle.getHistory())
    cycle.loadFromFile('work_cycle.json')
    print(cycle.getHistory())
    cycle.removeStep(0)
    print("after removing step 0: ",cycle.getHistory())
