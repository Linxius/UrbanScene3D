from pprint import pprint
from PIL import Image
import PIL
import piexif

codec = 'ISO-8859-1'  # or latin-1

def exif_to_tag(exif_dict):
    exif_tag_dict = {}
    thumbnail = exif_dict.pop('thumbnail')
    exif_tag_dict['thumbnail'] = thumbnail.decode(codec)

    for ifd in exif_dict:
        exif_tag_dict[ifd] = {}
        for tag in exif_dict[ifd]:
            try:
                element = exif_dict[ifd][tag].decode(codec)

            except AttributeError:
                element = exif_dict[ifd][tag]

            exif_tag_dict[ifd][piexif.TAGS[ifd][tag]["name"]] = element

    return exif_tag_dict

def main():
    # filename = 'IMG_2685.jpg'  # obviously one of your own pictures
    filename = 'DJI_0001.JPG'  # obviously one of your own pictures
    im = Image.open(filename)

    exif_dict = piexif.load(im.info.get('exif'))
    exif = {
        PIL.ExifTags.TAGS[k]: v
        for k, v in im._getexif().items()
        if k in PIL.ExifTags.TAGS
    }
    import pdb; pdb.set_trace()
    exif_dict = exif_to_tag(exif_dict)

    pprint(exif_dict['GPS'])

if __name__ == '__main__':
   main()