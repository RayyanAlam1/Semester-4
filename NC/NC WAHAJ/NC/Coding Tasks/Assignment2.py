# 20K-0208 (WAHAJ JAVED ALAM)
# SECTION : BCS - 4E
from math import *
from numpy import log
# ===========================================================================
#  NUMERICAL DIFFERENTIATION
# ===========================================================================
def three_point():
    x = []
    fx = []
    for i in range(4):
        a = float(input("Enter value of x" + str(i) +": "))
        x.append(a)
        a = float(input("Enter value of fx" + str(i) +": "))
        fx.append(a)
    h = x[1] - x[0]
    results = []
    results.append(((-3*fx[0])+(4*fx[0+1])-fx[0+2])/(2*h))
    results.append((fx[1+1]-fx[1-1])/(2*h))
    results.append((fx[2+1]-fx[2-1])/(2*h))
    results.append(((-3*fx[3])+(4*fx[3-1])-fx[3-2])/(2*(-1*h)))
    print("Three Point Results: ")
    print(" x         fx          f'(x)")
    for i in range(4):
        print(str(x[i])+str("      ")+'{0:.6g}'.format(fx[i])+str("      ")+'{0:.6g}'.format(results[i]))

def five_point():
    x = []
    fx = []
    for i in range(5):
        a = float(input("Enter value of x" + str(i) +": "))
        x.append(a)
        a = float(input("Enter value of fx" + str(i) +": "))
        fx.append(a)
    h = x[1] - x[0]
    results = []
    results.append(((-25*fx[0])+(48*fx[0+1]-(36*fx[0+2])+(16*fx[0+3])-(3*fx[0+4])))/(12*h))
    results.append("NULL")
    results.append(((fx[2-2])-(8*fx[2-1])+(8*fx[2+1])-(fx[2+2]))/(12*h))
    results.append("NULL")
    results.append(((-25*fx[4])+(48*fx[4-1]-(36*fx[4-2])+(16*fx[4-3])-(3*fx[4-4])))/(12*(-1*h)))
    print("Five Point Results: ")
    print(" x         fx          f'(x)")
    for i in range(5):
        if results[i] == "NULL":
            print(str(x[i])+str("      ")+'{0:.6g}'.format(fx[i])+str("      ")+results[i])
        else:
            print(str(x[i])+str("      ")+'{0:.6g}'.format(fx[i])+str("      ")+'{0:.6g}'.format(results[i]))
# ===========================================================================
#  NUMERICAL INTEGRATION
# ===========================================================================
def evaluate_expression(expression, x):
    expression.replace('x',str(x))
    result = eval(expression)
    return result

def open_cotes():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    for i in range(4):
        result=0
        h = (b-a)/(i+2)
        if i == 0:
            x0 = a + h
            resx = evaluate_expression(function,x0)
            result = 2*h*resx
        elif i == 1:
            x0 = a + h
            x1 = a + (2*h)
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            result = (3*h)*(resx0+resx1)/2
        elif i == 2:
            x0 = a + h
            x1 = a + (2*h)
            x2 = a + (3*h)
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            resx2 = evaluate_expression(function,x2)
            result = (4*h)*((2*resx0)-resx1+(2*resx2))/3
        elif i == 3:
            x0 = a + h
            x1 = a + (2*h)
            x2 = a + (3*h)
            x3 = a + (4*h)
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            resx2 = evaluate_expression(function,x2)
            resx3 = evaluate_expression(function,x3)
            result = (5*h)*((11*resx0)+resx1+resx2+(11*resx3))/24
        print("Iteration #"+str(i)+": " + str(result))
    
def closed_cotes():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    for i in range(4):
        result=0
        h = (b-a)/(i+1)
        if i == 0:
            x0 = a
            x1 = a + h
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            result = (h)*(resx0+resx1)/2
        elif i == 1:
            x0 = a 
            x1 = a + h
            x2 = a + (2*h)
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            resx2 = evaluate_expression(function,x2)
            result = (h)*(resx0 + (4*resx1) + resx2)/3
        elif i == 2:
            x0 = a 
            x1 = a + h
            x2 = a + (2*h)
            x3 = a + (3*h)
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            resx2 = evaluate_expression(function,x2)
            resx3 = evaluate_expression(function,x3)
            result = (3*h)*(resx0+(3*resx1)+(3*resx2)+resx3)/8
        elif i == 3:
            x0 = a
            x1 = a + h
            x2 = a + (2*h)
            x3 = a + (3*h)
            x4 = a + (4*h)
            resx0 = evaluate_expression(function,x0)
            resx1 = evaluate_expression(function,x1)
            resx2 = evaluate_expression(function,x2)
            resx3 = evaluate_expression(function,x3)
            resx4 = evaluate_expression(function,x4)
            result = ((2*h)*((7*resx0)+(32*resx1)+(12*resx2)+(32*resx3)+(7*resx4)))/45
        print("Iteration #"+str(i+1)+": " + str(result))

def trapezoidal_rule():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    h = b - a
    resx0 = evaluate_expression(function,a)
    resx1 = evaluate_expression(function,b)
    result = (h)*(resx0+resx1)/2
    print("Result = " + str(result))

def simpson_rule():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    h = (b-a)/2
    x1 = a + h
    resx0 = evaluate_expression(function,a)
    resx1 = evaluate_expression(function,x1)
    resx2 = evaluate_expression(function,b)
    result = (h)*(resx0 + (4*resx1) + resx2)/3
    print("Result = " + str(result))
# ===========================================================================
#  DIVIDED DIFFERENCE
# ===========================================================================
def divided_difference():
    x = []
    y = []
    for i in range(5):
        a = float(input("Enter x"+str(i)+str(": ")))
        x.append(a)
        b = float(input("Enter y"+str(i)+str(": ")))
        y.append(b)
    D0 = []
    D1 = []
    D2 = []
    D3 = []
    for i in range(4):
        D0.append((y[i+1]-y[i])/(x[i+1]-x[i]))
    for i in range(3):
        D1.append((D0[i+1]-D0[i])/(x[i+2]-x[i]))
    for i in range(2):
        D2.append((D1[i+1]-D1[i])/(x[i+3]-x[i]))
    D3.append((D2[1]-D2[0])/(x[4]-x[0]))
    print(" x        y     D0     D02    D03    D04")
    for i in range(5):
        if i == 0:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.4g}'.format(D0[i])+str("      ")
            +'{0:.4g}'.format(D1[i])+str("      ")+'{0:.4g}'.format(D2[i])+str("      ")
            +'{0:.4g}'.format(D3[i]))
        elif i == 1:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.4g}'.format(D0[i])+str("      ")
            +'{0:.4g}'.format(D1[i])+str("      ")+'{0:.4g}'.format(D2[i]))
        elif i == 2:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.4g}'.format(D0[i])+str("      ")
            +'{0:.4g}'.format(D1[i]))
        elif i == 3:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.4g}'.format(D0[i]))
        elif i == 4:
            print(str(x[i])+str("      ")+str(y[i]))

def central_difference():
    x = []
    y = []
    for i in range(5):
        a = float(input("Enter x"+str(i)+str(": ")))
        x.append(a)
        b = float(input("Enter y"+str(i)+str(": ")))
        y.append(b)
    req = float(input("Enter the value to be calculated: "))
    D0 = []
    D1 = []
    D2 = []
    D3 = []
    for i in range(4):
        D0.append(y[i+1]-y[i])
    for i in range(3):
        D1.append(D0[i+1]-D0[i])
    for i in range(2):
        D2.append(D1[i+1]-D1[i])
    D3.append(D2[1]-D2[0])
    print(" x        y        D0        D02          D03           D04")
    for i in range(5):
        if i == 0:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.6g}'.format(D0[i])+str("      ")
            +'{0:.6g}'.format(D1[i])+str("      ")+'{0:.6g}'.format(D2[i])+str("      ")
            +'{0:.6g}'.format(D3[i]))
        elif i == 1:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.6g}'.format(D0[i])+str("      ")
            +'{0:.6g}'.format(D1[i])+str("      ")+'{0:.6g}'.format(D2[i]))
        elif i == 2:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.6g}'.format(D0[i])+str("      ")
            +'{0:.6g}'.format(D1[i]))
        elif i == 3:
            print(str(x[i])+str("      ")+str(y[i])+str("     ")+'{0:.6g}'.format(D0[i]))
        elif i == 4:
            print(str(x[i])+str("      ")+str(y[i]))
    h = x[1] - x[0]   
    p = (req - x[2])/h
    result = y[2] + (p)*((D0[1]+D0[2])/2) + (p*p)*(D1[1])/2 + (p*((p*p)-1)*((D2[0]+D2[1])/2))/6
    + ((p*p)*((p*p)-1)*(D3[0]))/24
    print("P("+str(req)+") = " + str(result))

# ===========================================================================
#  COMPOSITE INTEGRALS 
# ===========================================================================
def composite_trapezoidal():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    n = int(input("Enter the value of n: "))
    h = (b-a)/n
    x = []
    for i in range(1,n):
        x.append(a+(i*h))
    resa = evaluate_expression(function,a)
    resb = evaluate_expression(function,b)
    resx = [evaluate_expression(function,x[i]) for i in range(len(x))]
    result = resa + resb
    for i in range(len(resx)):
        result = result + (2*resx[i])
    result = h*(result / 2)
    print("Result = " + str(result))

def composite_simpson():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    n = int(input("Enter the value of n: "))
    h = (b-a)/n
    x1 = []
    x2 = []
    for i in range(1,(int(n/2))):
        x1.append(a+(2*i*h))
    for i in range(1,int(n/2)+1):
        x2.append(a+((2*i)-1)*h)
    resa = evaluate_expression(function,a)
    resb = evaluate_expression(function,b)
    resx1 = [evaluate_expression(function,x1[i]) for i in range(len(x1))]
    resx2 = [evaluate_expression(function,x2[i]) for i in range(len(x2))]
    result = resa + resb
    for i in range(len(resx1)):
        result = result + (2*resx1[i])
    for i in range(len(resx2)):
        result = result + (4*resx2[i])
    result = h*(result/3)
    print("Result = " + str(result))

def composite_midpoint():
    function = input("Enter the function: ")
    function = function.replace('^','**')
    a = float(input("Enter the lower limit: "))
    b = float(input("Enter the upper limit: "))
    n = int(input("Enter the value of n: "))
    h = (b-a)/(n+2)
    x = []
    for i in range(int(n/2)+1):
        x.append(a+((2*i)+1)*h)
    resx = [evaluate_expression(function,x[i]) for i in range(len(x))]
    result = 0
    for i in range(len(resx)):
        result = result + resx[i]
    result = 2*h*result
    print("Result = " + str(result))
