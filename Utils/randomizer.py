import random

def random_order():
    items = ["A", "F", "B"]
    random.shuffle(items)
    return items

# Example usage
for _ in range(105):
    print(random_order())
