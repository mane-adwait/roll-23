Source: https://github.com/auralius/fulldiff

Code obtained from Dr. Aldo Ferri (Georgia Institute of Technology) in
ME6441-Fall 2017.
----------------------------

11Aug2020 Adwait Mane:
Taking time derivative of constraint vector h. Make sure h = h(q), not h(z)
because fulldiff works a bit differently compared to how one would
calculate by hand.

h = h(q)
fulldiff(h,q) gives (dh/dq)*(dq/dt).

h = h(z), where z=[q; dq/dt] (or higher derivatives)
fulldiff(h,q) gives (dh/dz)*(dz/dt). Note that this is ~= (dh/dq)*(dq/dt).
