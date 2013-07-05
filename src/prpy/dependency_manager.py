import herbpy, rospkg, os, sys

def append_to_env(variable, new_path, separator=':'):
    paths = os.environ.get(variable, '')
    if paths:
        paths_list = paths.split(separator)
    else:
        paths_list = list()

    # Check if we're adding a duplicate entry.
    duplicate = False
    new_canonical_path = os.path.realpath(new_path)
    for path in paths_list: 
        canonical_path = os.path.realpath(path)
        if canonical_path == new_canonical_path:
            duplicate = True
            break

    # Add the new path to the environmental variable.
    if not duplicate:
        paths_list.insert(0, new_path)
        paths = separator.join(paths_list)
        os.environ[variable] = paths

    return paths 

def export_paths(pkg, package_name):
    try:
        packages = pkg.get_depends(package_name)
        packages.append(package_name)
    except rospkg.ResourceNotFound:
        return False

    plugin_paths = list()
    data_paths = list()
    for package in packages:
        # Load the package manifest.
        try:
            manifest = pkg.get_manifest(package)
        except rospkg.ResourceNotFound:
            return False

        # Process the OpenRAVE export tags. 
        for plugin_path in manifest.get_export('openrave', 'plugins'):
            plugin_paths = append_to_env('OPENRAVE_PLUGINS', plugin_path)

        for data_path in manifest.get_export('openrave', 'data'):
            data_paths = append_to_env('OPENRAVE_DATA', data_path)

        # Add the appropriate directories to PYTHONPATH.
        # TODO: This is a hack. Should I parsing a Python export instead?
        sys.path.append(pkg.get_path(package) + '/src')

    return True

def export(package_name):
    pkg = rospkg.RosPack()
    manifest = pkg.get_manifest(package_name)

    # Required dependencies.
    if not export_paths(pkg, package_name):
        raise Exception('Unable to load required dependencies.')

    missing_packages = list()
    for optional_package in manifest.get_export('openrave', 'optional'):
        if not export_paths(pkg, optional_package):
            missing_packages.append(optional_package)

    if missing_packages:
        missing_packages.sort()
        herbpy.logger.warning('Missing optional dependencies: %s', ' '.join(missing_packages))
    return missing_packages
