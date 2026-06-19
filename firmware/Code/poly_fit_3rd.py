def solve_linear(A, b):
    # Gaussian elimination with partial pivoting
    n = len(b)

    for i in range(n):
        # pivot
        max_row = i
        for r in range(i + 1, n):
            if abs(A[r][i]) > abs(A[max_row][i]):
                max_row = r

        A[i], A[max_row] = A[max_row], A[i]
        b[i], b[max_row] = b[max_row], b[i]

        pivot = A[i][i]
        if abs(pivot) < 1e-12:
            raise ValueError("Singular matrix")

        # normalize row
        for j in range(i, n):
            A[i][j] /= pivot
        b[i] /= pivot

        # eliminate
        for r in range(n):
            if r != i:
                factor = A[r][i]
                for j in range(i, n):
                    A[r][j] -= factor * A[i][j]
                b[r] -= factor * b[i]

    return b


def polyfit3(points):
    # points = [(x0, y0), (x1, y1), ...]
    # fits y = a*x^3 + b*x^2 + c*x + d

    S = [0.0] * 7   # sums of x^0 ... x^6
    T = [0.0] * 4   # sums of y*x^0 ... y*x^3

    for x, y in points:
        xp = 1.0
        for k in range(7):
            S[k] += xp
            xp *= x

        xp = 1.0
        for k in range(4):
            T[k] += y * xp
            xp *= x

    # normal equation matrix for [d, c, b, a]
    A = [
        [S[0], S[1], S[2], S[3]],
        [S[1], S[2], S[3], S[4]],
        [S[2], S[3], S[4], S[5]],
        [S[3], S[4], S[5], S[6]],
    ]

    coeff = solve_linear(A, T)

    d, c, b, a = coeff
    return a, b, c, d


def eval_poly3(x, coeff):
    a, b, c, d = coeff
    return ((a * x + b) * x + c) * x + d


'''
# example of use
points= [(0.3619257, 0.1), (0.6202584, 0.2), (0.8253747, 0.3), (1.002665, 0.4), (1.152421, 0.5), (1.311136, 0.6), (1.470127, 0.7)]
coeff = polyfit3(points)
print(coeff)

print(eval_poly3(0.4, coeff))

'''