# 20K-0208 (WAHAJ JAVED ALAM)
# SECTION : BCS - 4E
# example function input : x^3 - 7*x^2 + 8*x - 0.35
# how to input
# exponential = exp(x) , sin(x), cos(x), tan(x), log(x) = ln(x)
from math import *
import sympy
from numpy import log


def factorial(num):
    if num == 0 or num == 1:
        return 1
    else:
        return num * factorial(num-1)


def evaluate_expression(expression, x):
    result = eval(expression)
    return result


def bisection_method(function, iterations):
    a = float(input("Enter the lower bound: "))
    b = float(input("Enter the upper bound: "))
    fa = evaluate_expression(function, a)
    fb = evaluate_expression(function, b)
    print("Bisection Method")
    print("================")
    if fa * fb > 0:
        print("The solution is not possible.")
        return
    else:
        for i in range(iterations):
            c = float((a + b)) / 2.0
            fa = evaluate_expression(function, a)
            fc = evaluate_expression(function, c)
            if fa * fc < 0:
                b = c
            elif fa * fc > 0:
                a = c
            else:
                return
            print("Iteration # " + str(i + 1) + " : " + str(c))


def fixed_point_method(function, iterations):
    x0 = float(input("Enter the first value: "))
    print("Fixed Point Method")
    print("==================")
    for i in range(iterations):
        xn = evaluate_expression(function, x0)
        print("Iteration # " + str(i + 1) + " : " + str(xn))
        x0 = xn


def newton_method(function, iterations):
    derivative = sympy.diff(function)
    x0 = float(input("Enter the first value: "))
    print("Newton Method")
    print("=============")
    for i in range(iterations):
        xn = x0 - (evaluate_expression(function, x0) / derivative.subs("x", x0))
        print("Iteration # " + str(i + 1) + " : " + str(xn))
        x0 = xn


def secant_method(function, iterations):
    x0 = float(input("Enter the first value: "))
    x1 = float(input("Enter the second value: "))
    print("Secant Method")
    print("=============")
    for i in range(iterations):
        xn = x1 - (float((evaluate_expression(function, x1) * (x1 - x0))) / float(
            evaluate_expression(function, x1) - evaluate_expression(function, x0)))
        print("Iteration # " + str(i + 1) + " : " + str(xn))
        x0 = x1
        x1 = xn


def false_position_method(function, iterations):
    a = float(input("Enter the lower bound: "))
    b = float(input("Enter the upper bound: "))
    fa = evaluate_expression(function, a)
    fb = evaluate_expression(function, b)
    print("False Position Method")
    print("=====================")
    if fa * fb > 0:
        print("The solution is not possible.")
        return
    else:
        for i in range(iterations):
            fa = evaluate_expression(function, a)
            fb = evaluate_expression(function, b)
            c = (((a * fb) - (b * fa)) / (fb - fa))
            fc = evaluate_expression(function, c)
            if fa * fc < 0:
                b = c
            elif fa * fc > 0:
                a = c
            else:
                return
            print("Iteration # " + str(i + 1) + " : " + str(c))


def relative_error(function):
    x = float(input("Enter the value of x: "))
    true_val = evaluate_expression(function, x)
    approx_val = float(input("Enter the approximated value of the function: "))
    rel_error = abs(true_val - approx_val)
    abs_error = rel_error/true_val
    percentage_abs_rel_error = abs_error * 100
    print("The Relative Error at x = " + str(x) + " is " + str(rel_error))
    print("The Absolute Relative Error at x = " + str(x) + " is " + str(percentage_abs_rel_error) + " % ")


def taylor_polynomial(function):
    n = int(input("Enter the degree of polynomial: "))
    x = float(input("Enter the value of x: "))
    derivatives = []
    fx0 = [evaluate_expression(function, x)]
    check = function
    answer = ""
    for i in range(n+1):
        derivatives.append(sympy.diff(check))
        check = derivatives[i]
        fx0.append(derivatives[i].subs("x", x))
    for i in range(n+1):
        if fx0[i] == 0:
            continue
        else:
            if i == 0:
                answer = answer + str(fx0[i])
            else:
                if x == 0:
                    answer = answer + " + (" + str(fx0[i]) + "(x) ^ " + str(i) + ")/" + str(factorial(i))+" "
                else:
                    answer = answer + " + (" + str(fx0[i]) + "( x - " + str(x) + ") ^ " + str(i) + ")/" + str(factorial(i))+" "
    print(answer)


def lagrange_interpolation():
    ch = int(input("Solve by 1) Function 2) Values"))
    n = int(input("Enter the degree: "))
    x = float(input("Enter the value of x: "))
    yy = []
    xx = []
    lx = []
    bound_error = 1
    if ch == 1:
        function = input("Enter the function: ")
        function = function.replace('^', '**')
        for i in range(n+1):
            x0 = float(input("Enter the value of x0: "))
            xx.append(x0)
            yy.append(evaluate_expression(function,x0))
        diff = sympy.diff(function)
        for j in range(n):
            diff = sympy.diff(diff)
        gx = float(input("Enter the value of x from g(x): "))
        for j in range(n+1):
            bound_error = bound_error * (gx - xx[j])
        bound_error = bound_error * diff.subs("x", xx[0])
        bound_error = bound_error / factorial(n+1)
    else:
        for i in range(n + 1):
            x0 = float(input("Enter the value of x0: "))
            y0 = float(input("Enter the value of y0: "))
            xx.append(x0)
            yy.append(y0)

    for i in range(n+1):
        ans = 1
        for j in range(n+1):
            if i == j:
                continue
            else:
                ans = ans * (x - xx[j])
        for j in range(n+1):
            if i==j:
                continue
            else:
                ans = ans / (xx[i] - xx[j])
        lx.append(ans)
    final = 0
    for i in range(len(lx)):
        final = final + lx[i]*yy[i]
    print("P(" + str(x) + ") = " + str(final))
    print("Bound error: " + str(bound_error))


if __name__ == '__main__':
    choice = int(input("""
    Which method do you want to use: 
    (in case of lagrange interpolation do not enter the function write anything and press enter)
    1) Relative Error
    2) Taylor Polynomial
    3) Bisection Method
    4) Fixed Point Method
    5) Newton Method
    6) Secant Method
    7) False Position Method 
    8) Lagrange Interpolation : """))
    string = input("Enter the function: ")
    string = string.replace('^', '**')
    if choice == 1:
        relative_error(string)
    elif choice == 2:
        taylor_polynomial(string)
    elif choice == 3:
        iteration_count = int(input("Enter the number of iterations: "))
        bisection_method(string, iteration_count)
    elif choice == 4:
        iteration_count = int(input("Enter the number of iterations: "))
        fixed_point_method(string, iteration_count)
    elif choice == 5:
        iteration_count = int(input("Enter the number of iterations: "))
        newton_method(string, iteration_count)
    elif choice == 6:
        iteration_count = int(input("Enter the number of iterations: "))
        secant_method(string, iteration_count)
    elif choice == 7:
        iteration_count = int(input("Enter the number of iterations: "))
        false_position_method(string, iteration_count)
    elif choice == 8:
        lagrange_interpolation()
