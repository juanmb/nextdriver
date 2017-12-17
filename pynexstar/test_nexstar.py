from math import pi
import ephem
import unittest
from nexstar import NexStar, RA_DEV, DEC_DEV

PORT = '/dev/ttyUSB0'


class TestNexStar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nex = NexStar(PORT, False)

    @classmethod
    def tearDownClass(cls):
        cls.nex.close()

    def setUp(self):
        self.obs = ephem.city('Madrid')
        self.obs.date = ephem.Date('2017/12/11 22:02:33')
        self.obs.pressure = 0.0

        self.nex.set_location(self.obs.lat, self.obs.long)
        self.nex.set_time(str(self.obs.date))

    def test_echo(self):
        c = self.nex.echo('x')
        self.assertEqual('x', c)

    def test_versions(self):
        self.assertEqual('4.21', self.nex.get_version())
        self.assertEqual('5.07', self.nex.get_device_version(RA_DEV))
        self.assertEqual('5.07', self.nex.get_device_version(DEC_DEV))

    def test_model(self):
        self.assertEqual(10, self.nex.get_model())

    def test_location(self):
        lat, lon = self.nex.get_location()
        self.assertAlmostEqual(self.obs.lat.real, lat, 5)
        self.assertAlmostEqual(self.obs.lon.real, lon, 5)

    def test_julian_date(self):
        jd1 = ephem.julian_date(self.obs.date) - 2451545.0
        jd2 = self.nex.get_j2000_date()
        self.assertAlmostEqual(jd1, jd2, 4)

    def test_sidereal_time(self):
        lst1 = self.obs.sidereal_time()

        self.nex.set_time(str(self.obs.date))
        lst2 = self.nex.get_sidereal_time()

        # difference should be < 10 seconds
        self.assertLess(abs(lst1 - lst2), 10*pi/43200)

    def test_eq_coords(self):
        stars = ['Polaris', 'Dubhe', 'Merak', 'Deneb', 'Algol', 'Vega',
                 'Altair', 'Rigel', 'Spica', 'Sirius', 'Achernar', 'Canopus']

        for star in stars:
            star = ephem.star(star)
            star.compute(self.obs)

            self.nex.sync_eq_coords(star.ra, star.dec, precise=True)

            ra, dec = self.nex.get_eq_coords(precise=False)
            self.assertAlmostEqual(star.ra, ra, 3)
            self.assertAlmostEqual(star.dec, dec, 3)

            ra, dec = self.nex.get_eq_coords(precise=True)
            self.assertAlmostEqual(star.ra, ra, 5)
            self.assertAlmostEqual(star.dec, dec, 5)

    def test_az_coords(self):
        stars = ['Polaris', 'Dubhe', 'Merak', 'Deneb', 'Algol', 'Vega',
                 'Altair', 'Rigel', 'Spica', 'Sirius', 'Achernar', 'Canopus']

        for star in stars:
            star = ephem.star(star)
            star.compute(self.obs)

            self.nex.sync_eq_coords(star.ra, star.dec, precise=True)

            az, alt = self.nex.get_az_coords(precise=False)
            self.assertAlmostEqual(star.az, az, 2)
            self.assertAlmostEqual(star.alt, alt, 2)

            az, alt = self.nex.get_az_coords(precise=True)
            self.assertAlmostEqual(star.az, az, 3)
            self.assertAlmostEqual(star.alt, alt, 3)



if __name__ == '__main__':
    unittest.main()
