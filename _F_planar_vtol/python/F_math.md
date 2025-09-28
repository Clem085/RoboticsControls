# D.3 Mass–Spring–Damper: Equations of Motion

**System:** point mass with spring \(k\), damper \(b\), position \(z\), external force \(F\).

---

### (a) Potential energy
$$
P(z)=\frac{1}{2}\,k\,z^{2}.
$$

### (b) Generalized coordinate
$$
q=z.
$$

### (c) Generalized and damping forces
Generalized force along \(z\): \( \tau = F \).

Model viscous damping with the Rayleigh dissipation function
$$
\mathcal{R}=\frac{1}{2}\,b\,\dot z^{2}
\quad\Rightarrow\quad
Q_{\text{damp}}=-\frac{\partial \mathcal{R}}{\partial \dot z}=-b\,\dot z.
$$
Total nonconservative generalized force:
$$
Q = \tau + Q_{\text{damp}} = F - b\,\dot z.
$$

### (d) Euler–Lagrange with dissipation
Kinetic energy \(K=\tfrac{1}{2} m \dot z^{2}\), Lagrangian \(L=K-P\).
The Euler–Lagrange equation with nonconservative force \(Q\) is
$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot z}\right)
-\frac{\partial L}{\partial z}= Q .
$$
Computing terms:
\[
\frac{\partial L}{\partial \dot z}= m\,\dot z,
\qquad
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot z}\right)= m\,\ddot z,
\qquad
\frac{\partial L}{\partial z}= -k\,z.
\]
Therefore
$$
m\,\ddot z + k\,z = F - b\,\dot z.
$$

### Result (equation of motion)
$$
\boxed{\,m\,\ddot z + b\,\dot z + k\,z = F\, }.
$$
