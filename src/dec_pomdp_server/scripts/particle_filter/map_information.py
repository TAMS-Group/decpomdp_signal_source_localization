# Might help me in the future

class MapInformation(object):
    """docstring forMapInformation."""

    def __init__(self):
        super(MapInformation, self).__init__()
        self.mapfile = open('tams_localization.pgm')
        self.mapinfo = self.read_pgm(self.mapfile)
        self.mapfile.close()

    def read_pgm(self, pgmf):
        """Return a raster of integers from a PGM as a list of lists."""
        assert pgmf.readline() == 'P5\n'
        (width, height) = [int(i) for i in pgmf.readline().split()]
        depth = int(pgmf.readline())
        assert depth <= 255

        raster = []
        for y in range(height):
            row = []
            for y in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
        return raster

if __name__ == '__main__':
    mapinfo = MapInformation()
    print(mapinfo.mapinfo)
