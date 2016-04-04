import hashlib
import os.path
import shutil


class SerializationDatabase(object):
    def __init__(self, path):
        self.path = path

    def get_key(self, source_path, extension=None):
        if extension is None:
            _, extension = os.path.splitext(source_path)

        return self._get_hash(source_path) + extension

    def get_path(self, key, verify=True):
        resource_path = os.path.join(self.path, key)

        if verify:
            key_hash, _ = os.path.splitext(key)
            resource_hash = self._get_hash(resource_path)

            if resource_hash != key_hash:
                raise IOError(
                    'Hash mismatch "{:s}" has hash {:s}; expected {:s}.'
                    .format(resource_path, resource_hash, key_hash))

        return resource_path

    def save(self, source_path, extension=None):
        key = self.get_key(source_path, extension)
        shutil.copyfile(source_path, self.get_path(key))
        return key

    def _get_hash(self, path):
        with open(source_path, 'rb') as source_file:
            source_content = source_file.read()

        return hashlib.md5(source_path)

def serialize_kinbody(kinbody, database):
    filenames = robot.GetURI()
    if not filenames:
        raise ValueError(
            'Unable to serialize KinBody with empty GetXMLFileName().')

    return {
        'XMLId': kinbody.GetXMLId(),
        'URI': [database.save(path) for path in filenames.split()],
        'KinematicsGeometryHash': kinbody.GetKinematicsGeometryHash(),
        'RobotStructureHash': kinbody.GetRobotStructureHash(),
    }


    for 

