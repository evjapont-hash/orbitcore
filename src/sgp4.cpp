// Near-Earth SGP4 propagator.
//
// The algorithm and coefficient names follow the reference implementation in:
//   Vallado, D. A., Crawford, P., Hujsak, R., and Kelso, T. S.,
//   "Revisiting Spacetrack Report #3", AIAA 2006-6753.
//   Source code and test cases: https://celestrak.org/publications/AIAA/2006-6753/
// Deep-space (SDP4) terms are not implemented.
#include "orbitcore/sgp4.hpp"

#include <cmath>

namespace orbitcore {

namespace {

// Converts the TLE's "Kozai" mean motion to the "Brouwer" mean motion that SGP4 works with.
// (Same idea as Vallado's `initl`.) Returns rad/min.
double brouwer_mean_motion(const ElementSet& e, const GravityModel& g) {
  const double x2o3 = 2.0 / 3.0;
  const double cosio = std::cos(e.inclo);
  const double cosio2 = cosio * cosio;
  const double omeosq = 1.0 - e.ecco * e.ecco;
  const double rteosq = std::sqrt(omeosq);

  const double ak = std::pow(g.xke / e.no_kozai, x2o3);
  const double d1 = 0.75 * g.j2 * (3.0 * cosio2 - 1.0) / (rteosq * omeosq);
  double del = d1 / (ak * ak);
  const double adel = ak * (1.0 - del * del - del * (1.0 / 3.0 + 134.0 * del * del / 81.0));
  del = d1 / (adel * adel);
  return e.no_kozai / (1.0 + del);
}

}  // namespace

bool is_deep_space(const ElementSet& e) {
  if (!(e.no_kozai > 0.0)) return false;
  return (kTwoPi / brouwer_mean_motion(e, wgs72())) >= 225.0;
}

Sgp4::Sgp4(const ElementSet& elements, const GravityModel& grav)
    : elements_(elements), grav_(grav) {
  const double j2 = grav_.j2, j3oj2 = grav_.j3oj2, j4 = grav_.j4;
  const double re = grav_.radius_earth_km;
  const double xke = grav_.xke;
  const double x2o3 = 2.0 / 3.0;
  const double temp4 = 1.5e-12;

  const double ecco = elements_.ecco;
  const double inclo = elements_.inclo;
  const double argpo = elements_.argpo;
  const double mo = elements_.mo;
  const double bstar = elements_.bstar;

  if (!(elements_.no_kozai > 0.0)) { init_status_ = Sgp4Error::MeanMotion; return; }
  if (ecco < 0.0 || ecco >= 1.0) { init_status_ = Sgp4Error::MeanElements; return; }

  // ---- Step 1: basic quantities derived from the TLE (Vallado's `initl`) ----
  const double eccsq = ecco * ecco;
  const double omeosq = 1.0 - eccsq;
  const double rteosq = std::sqrt(omeosq);
  const double cosio = std::cos(inclo);
  const double sinio = std::sin(inclo);
  const double cosio2 = cosio * cosio;

  const double no = brouwer_mean_motion(elements_, grav_);  // rad/min
  c_.no = no;
  if (kTwoPi / no >= 225.0) { init_status_ = Sgp4Error::DeepSpaceUnsupported; return; }

  const double ao = std::pow(xke / no, x2o3);  // semi-major axis, earth radii
  const double po = ao * omeosq;
  const double con42 = 1.0 - 5.0 * cosio2;
  c_.con41 = -con42 - cosio2 - cosio2;
  const double posq = po * po;
  const double rp = ao * (1.0 - ecco);  // perigee radius, earth radii

  // ---- Step 2: choose the atmospheric drag model from the perigee height ----
  c_.isimp = rp < (220.0 / re + 1.0);
  double sfour = 78.0 / re + 1.0;
  double qzms24 = std::pow((120.0 - 78.0) / re, 4.0);
  const double perige = (rp - 1.0) * re;  // km above the surface
  if (perige < 156.0) {
    sfour = perige - 78.0;
    if (perige < 98.0) sfour = 20.0;
    qzms24 = std::pow((120.0 - sfour) / re, 4.0);
    sfour = sfour / re + 1.0;
  }
  const double pinvsq = 1.0 / posq;

  // ---- Step 3: drag and secular coefficients ----
  const double tsi = 1.0 / (ao - sfour);
  c_.eta = ao * ecco * tsi;
  const double eta = c_.eta;
  const double etasq = eta * eta;
  const double eeta = ecco * eta;
  const double psisq = std::fabs(1.0 - etasq);
  const double coef = qzms24 * std::pow(tsi, 4.0);
  const double coef1 = coef / std::pow(psisq, 3.5);
  const double cc2 =
      coef1 * no *
      (ao * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) +
       0.375 * j2 * tsi / psisq * c_.con41 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
  c_.cc1 = bstar * cc2;
  double cc3 = 0.0;
  if (ecco > 1.0e-4) cc3 = -2.0 * coef * tsi * j3oj2 * no * sinio / ecco;
  c_.x1mth2 = 1.0 - cosio2;
  c_.cc4 = 2.0 * no * coef1 * ao * omeosq *
           (eta * (2.0 + 0.5 * etasq) + ecco * (0.5 + 2.0 * etasq) -
            j2 * tsi / (ao * psisq) *
                (-3.0 * c_.con41 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta)) +
                 0.75 * c_.x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * std::cos(2.0 * argpo)));
  c_.cc5 = 2.0 * coef1 * ao * omeosq * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);

  // ---- Step 4: secular rates (how fast the angles drift because Earth is not a sphere) ----
  const double cosio4 = cosio2 * cosio2;
  const double temp1 = 1.5 * j2 * pinvsq * no;
  const double temp2 = 0.5 * temp1 * j2 * pinvsq;
  const double temp3 = -0.46875 * j4 * pinvsq * pinvsq * no;
  c_.mdot = no + 0.5 * temp1 * rteosq * c_.con41 +
            0.0625 * temp2 * rteosq * (13.0 - 78.0 * cosio2 + 137.0 * cosio4);
  c_.argpdot = -0.5 * temp1 * con42 + 0.0625 * temp2 * (7.0 - 114.0 * cosio2 + 395.0 * cosio4) +
               temp3 * (3.0 - 36.0 * cosio2 + 49.0 * cosio4);
  const double xhdot1 = -temp1 * cosio;
  c_.nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * cosio2) + 2.0 * temp3 * (3.0 - 7.0 * cosio2)) * cosio;
  c_.omgcof = bstar * cc3 * std::cos(argpo);
  c_.xmcof = 0.0;
  if (ecco > 1.0e-4) c_.xmcof = -x2o3 * coef * bstar / eeta;
  c_.nodecf = 3.5 * omeosq * xhdot1 * c_.cc1;
  c_.t2cof = 1.5 * c_.cc1;
  if (std::fabs(cosio + 1.0) > 1.5e-12)
    c_.xlcof = -0.25 * j3oj2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
  else
    c_.xlcof = -0.25 * j3oj2 * sinio * (3.0 + 5.0 * cosio) / temp4;
  c_.aycof = -0.5 * j3oj2 * sinio;
  const double delmotemp = 1.0 + eta * std::cos(mo);
  c_.delmo = delmotemp * delmotemp * delmotemp;
  c_.sinmao = std::sin(mo);
  c_.x7thm1 = 7.0 * cosio2 - 1.0;

  // Higher-order drag terms, only used when perigee is high enough (not the "simple" model).
  if (!c_.isimp) {
    const double cc1sq = c_.cc1 * c_.cc1;
    c_.d2 = 4.0 * ao * tsi * cc1sq;
    const double temp = c_.d2 * tsi * c_.cc1 / 3.0;
    c_.d3 = (17.0 * ao + sfour) * temp;
    c_.d4 = 0.5 * temp * ao * tsi * (221.0 * ao + 31.0 * sfour) * c_.cc1;
    c_.t3cof = c_.d2 + 2.0 * cc1sq;
    c_.t4cof = 0.25 * (3.0 * c_.d3 + c_.cc1 * (12.0 * c_.d2 + 10.0 * cc1sq));
    c_.t5cof = 0.2 * (3.0 * c_.d4 + 12.0 * c_.cc1 * c_.d3 + 6.0 * c_.d2 * c_.d2 +
                      15.0 * cc1sq * (2.0 * c_.d2 + cc1sq));
  }

    init_status_ = Sgp4Error::None;  // initialization succeeded
}

PropagationResult Sgp4::propagate(double tsince) const {
  if (init_status_ != Sgp4Error::None) return {init_status_, {}};

  const double xke = grav_.xke;
  const double re = grav_.radius_earth_km;
  const double j2 = grav_.j2;
  const double x2o3 = 2.0 / 3.0;
  const double vkmpersec = re * xke / 60.0;  // converts earth-radii/min to km/s
  const Coeffs& c = c_;
  const ElementSet& e = elements_;

  // ---- Step 1: secular gravity and atmospheric drag: advance the mean elements ----
  const double xmdf = e.mo + c.mdot * tsince;
  const double argpdf = e.argpo + c.argpdot * tsince;
  const double nodedf = e.nodeo + c.nodedot * tsince;
  double argpm = argpdf;
  double mm = xmdf;
  const double t2 = tsince * tsince;
  double nodem = nodedf + c.nodecf * t2;
  double tempa = 1.0 - c.cc1 * tsince;
  double tempe = e.bstar * c.cc4 * tsince;
  double templ = c.t2cof * t2;

  if (!c.isimp) {
    const double delomg = c.omgcof * tsince;
    const double delmtemp = 1.0 + c.eta * std::cos(xmdf);
    const double delm = c.xmcof * (delmtemp * delmtemp * delmtemp - c.delmo);
    const double temp = delomg + delm;
    mm = xmdf + temp;
    argpm = argpdf - temp;
    const double t3 = t2 * tsince;
    const double t4 = t3 * tsince;
    tempa = tempa - c.d2 * t2 - c.d3 * t3 - c.d4 * t4;
    tempe = tempe + e.bstar * c.cc5 * (std::sin(mm) - c.sinmao);
    templ = templ + c.t3cof * t3 + t4 * (c.t4cof + tsince * c.t5cof);
  }

  double nm = c.no;
  double em = e.ecco;
  const double inclm = e.inclo;

  if (nm <= 0.0) return {Sgp4Error::MeanMotion, {}};

  const double am = std::pow(xke / nm, x2o3) * tempa * tempa;  // semi-major axis, earth radii
  nm = xke / std::pow(am, 1.5);
  em = em - tempe;

  if (em >= 1.0 || em < -0.001) return {Sgp4Error::MeanElements, {}};
  if (em < 1.0e-6) em = 1.0e-6;
  mm = mm + c.no * templ;
  double xlm = mm + argpm + nodem;

  nodem = std::fmod(nodem, kTwoPi);
  argpm = std::fmod(argpm, kTwoPi);
  xlm = std::fmod(xlm, kTwoPi);
  mm = std::fmod(xlm - argpm - nodem, kTwoPi);

  const double sinim = std::sin(inclm);
  const double cosim = std::cos(inclm);

  // (Deep-space lunar/solar terms would be added here; orbitcore is near-Earth only.)
  const double ep = em, xincp = inclm, argpp = argpm, nodep = nodem, mp = mm;
  const double sinip = sinim, cosip = cosim;
  if (ep < 0.0 || ep > 1.0) return {Sgp4Error::PerturbedElements, {}};

  // ---- Step 2: long-period periodics, then solve Kepler's equation ----
  const double axnl = ep * std::cos(argpp);
  double temp = 1.0 / (am * (1.0 - ep * ep));
  const double aynl = ep * std::sin(argpp) + temp * c.aycof;
  const double xl = mp + argpp + nodep + temp * c.xlcof * axnl;

  const double u = std::fmod(xl - nodep, kTwoPi);
  double eo1 = u;
  double tem5 = 9999.9;
  double sineo1 = 0.0, coseo1 = 0.0;
  for (int ktr = 1; std::fabs(tem5) >= 1.0e-12 && ktr <= 10; ++ktr) {
    sineo1 = std::sin(eo1);
    coseo1 = std::cos(eo1);
    tem5 = 1.0 - coseo1 * axnl - sineo1 * aynl;
    tem5 = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
    if (std::fabs(tem5) >= 0.95) tem5 = tem5 > 0.0 ? 0.95 : -0.95;
    eo1 += tem5;
  }

  // ---- Step 3: short-period quantities ----
  const double ecose = axnl * coseo1 + aynl * sineo1;
  const double esine = axnl * sineo1 - aynl * coseo1;
  const double el2 = axnl * axnl + aynl * aynl;
  const double pl = am * (1.0 - el2);
  if (pl < 0.0) return {Sgp4Error::SemiLatusRectum, {}};

  const double rl = am * (1.0 - ecose);
  const double rdotl = std::sqrt(am) * esine / rl;
  const double rvdotl = std::sqrt(pl) / rl;
  const double betal = std::sqrt(1.0 - el2);
  temp = esine / (1.0 + betal);
  const double sinu = am / rl * (sineo1 - aynl - axnl * temp);
  const double cosu = am / rl * (coseo1 - axnl + aynl * temp);
  double su = std::atan2(sinu, cosu);
  const double sin2u = (cosu + cosu) * sinu;
  const double cos2u = 1.0 - 2.0 * sinu * sinu;
  temp = 1.0 / pl;
  const double temp1 = 0.5 * j2 * temp;
  const double temp2 = temp1 * temp;

  const double mrt = rl * (1.0 - 1.5 * temp2 * betal * c.con41) + 0.5 * temp1 * c.x1mth2 * cos2u;
  su = su - 0.25 * temp2 * c.x7thm1 * sin2u;
  const double xnode = nodep + 1.5 * temp2 * cosip * sin2u;
  const double xinc = xincp + 1.5 * temp2 * cosip * sinip * cos2u;
  const double mvt = rdotl - nm * temp1 * c.x1mth2 * sin2u / xke;
  const double rvdot = rvdotl + nm * temp1 * (c.x1mth2 * cos2u + 1.5 * c.con41) / xke;

  // ---- Step 4: orientation vectors -> position and velocity in TEME ----
  const double sinsu = std::sin(su), cossu = std::cos(su);
  const double snod = std::sin(xnode), cnod = std::cos(xnode);
  const double sini = std::sin(xinc), cosi = std::cos(xinc);
  const double xmx = -snod * cosi;
  const double xmy = cnod * cosi;
  const double ux = xmx * sinsu + cnod * cossu;
  const double uy = xmy * sinsu + snod * cossu;
  const double uz = sini * sinsu;
  const double vx = xmx * cossu - cnod * sinsu;
  const double vy = xmy * cossu - snod * sinsu;
  const double vz = sini * cossu;

  if (mrt < 1.0) return {Sgp4Error::Decayed, {}};  // inside the Earth: satellite has decayed

  PropagationResult out;
  out.error = Sgp4Error::None;
  out.state.r = {mrt * ux * re, mrt * uy * re, mrt * uz * re};
  out.state.v = {(mvt * ux + rvdot * vx) * vkmpersec, (mvt * uy + rvdot * vy) * vkmpersec,
                 (mvt * uz + rvdot * vz) * vkmpersec};
  return out;
}

PropagationResult Sgp4::propagate_to(JulianDate t) const {
  return propagate(minutes_between(elements_.epoch, t));
}

}  // namespace orbitcore