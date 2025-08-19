class PersonNode:    #创建类（首字母大写）
    def __init__(self,name:str,age:int) -> None:    #定义init方法（在创建类的时候调用）
        print("__init__方法被调用")
        self.name = name    #self：类本身
        self.age = age

    def eat(self,food_name:str):    #类的方法，第一个变量必须是self
        """
        说明：
        方法：吃东西
        food_name：实物名字
        """
        print(f"{self.name}，{self.age}岁，爱吃{food_name}")    #f：格式化
        
def main():
    node = PersonNode('法外狂徒张三',18)    #创建类实例
    node.eat('鱼香肉丝')