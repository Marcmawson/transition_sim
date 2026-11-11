#ifndef VOF_COMPENSATED_INCLUDED
#define VOF_COMPENSATED_INCLUDED

// A scalar is represented as the sum of a leading FP32 word and a
// non-overlapping residual word. Precise prevents contraction/reassociation
// of the error-free transforms used by the conservation ledger.
float2 V(float x) { return float2(x, 0); }
float2 NormalizePair(float hi, float lo)
{
    precise float s = hi + lo;
    precise float r = lo - (s - hi);
    return float2(s, r);
}
float2 Add(float2 a, float2 b)
{
    precise float s = a.x + b.x;
    precise float v = s - a.x;
    precise float e = (a.x - (s - v)) + (b.x - v);
    precise float t = (e + a.y) + b.y;
    return NormalizePair(s, t);
}
float2 Sub(float2 a, float2 b) { return Add(a, -b); }
float2 Split(float x)
{
    precise float c = 4097.0 * x;
    precise float h = c - (c - x);
    precise float l = x - h;
    return float2(h, l);
}
float2 Mul(float2 a, float2 b)
{
    precise float p = a.x * b.x;
    precise float2 x = Split(a.x), y = Split(b.x);
    precise float e = ((x.x * y.x - p) + x.x * y.y + x.y * y.x) + x.y * y.y;
    precise float t = (e + a.x * b.y) + a.y * b.x;
    return NormalizePair(p, t);
}
float2 Div(float2 a, float2 b)
{
    precise float q = a.x / b.x;
    precise float2 rem = Sub(a, Mul(b, V(q)));
    precise float correction = (rem.x + rem.y) / b.x;
    return NormalizePair(q, correction);
}
bool Less(float2 a, float2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); }
bool Equal(float2 a, float2 b) { return a.x == b.x && a.y == b.y; }
float2 MinPair(float2 a, float2 b) { return Less(a, b) ? a : b; }
float2 MaxPair(float2 a, float2 b) { return Less(a, b) ? b : a; }
float2 AbsPair(float2 a) { return Less(a, V(0)) ? -a : a; }
float2 Bounded(float2 a, float2 high) { return MinPair(MaxPair(a, V(0)), high); }
#endif
