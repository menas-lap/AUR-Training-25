from time import time
from rich.console import Console

console = Console()
def send_Msg(msg):
  if isinstance(msg, BaseMsg):
    console.print(msg, style=msg.style)
  else:
    print(msg)


class BaseMsg:
  def __init__(self, data: str):
    self._data = data
  
  @property
  def style(self):
    return '' # BaseMsg-specific
      
  @property
  def data(self):
    return self._data
  
  def __str__(self):
    return self._data # BaseMsg-specific
  
  def __len__(self):
    return len(str(self))
  
  def __eq__(self, other):
    if isinstance(other, BaseMsg):
        return str(self) == str(other)
    return False
  
  def __add__(self, other):
      if isinstance(other, BaseMsg):
         return self.__class__(str(self) + " " + str(other))
      elif isinstance(other, str):
         return self.__class__(str(self) + " " + other)
      return NotImplemented


class LogMsg(BaseMsg):
  def __init__(self, data):
    super().__init__(data)
    self._timestamp = int(time())

  @property  
  def style(self):
    return "yellow"
  
  def __str__(self):
    return f"[{self._timestamp}] {self.data}"


class WarnMsg(LogMsg):
  @property
  def style(self):
    return "red"
  
  def __str__(self):
    return f"[!WARN][{self._timestamp}] {self.data}"

if __name__ == '__main__':
    m1 = BaseMsg('\nNormal message\n')
    m2 = LogMsg('Log\n')
    m3 = WarnMsg('Warning\n')
    send_Msg(m1)
    send_Msg(m2)
    send_Msg(m3)