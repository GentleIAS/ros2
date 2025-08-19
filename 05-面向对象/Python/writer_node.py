from python_package.person_node import PersonNode    #导入python_package文件夹下的person_node的PersonNode类

class WriterNode(PersonNode):
    #name无传入时默认“匿名作者”，age：18，write：没有作品
    def __init__(self,name:str = '匿名作者',age:int = 18,write:str = '没有作品') -> None:
        self.write = write
        super().__init__(name,age)

    def eat(self,food_name:str):    #重写eat方法
        """
        说明：
        方法：吃东西
        food_name：实物名字
        """
        print(f"{self.name}，{self.age}岁，爱吃：{food_name}，书籍：{self.write}")

def main():
    node1 = WriterNode(name = '村民李四',age = 35)
    node2 = WriterNode(write = '在ros2速成python')
    node1.eat('香蕉')
    node2.eat('苹果')

