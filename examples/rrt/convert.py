with open("./cac.path", "r") as f:
    s = ""
    for line in f.readlines():
        n = line.split()
        x = float(n[0]) / 100.0
        y = float(n[1]) / 100.0
        print(f"({x}, {y}),")
