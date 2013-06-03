import numpy

class ConfigurationLibrary:
    def __init__(self):
        self._indices = set()
        self._groups = dict()
        self._configs = dict()

    def load_yaml(self, path):
        with open(path, 'rb') as config_file:
            import yaml
            config_yaml = yaml.load(config_file)

        for name, groups in config_yaml['configurations'].items():
            self.add_configuration(name, **groups)

    def add_group(self, name, dof_indices):
        duplicate_indices = self._indices.intersection(dof_indices)

        if name in self._groups:
            raise Exception('There is already a group named %s.' % name)
        elif duplicate_indices:
            sorted_duplicates = sorted(list(duplicate_indices))
            raise Exception('Indices [ %s ] are already bound to names.'\
                            % ', '.join(map(str, sorted_duplicates)))

        self._groups[name] = numpy.array(dof_indices).copy()
        self._indices |= set(dof_indices)

    def add_configuration(self, name, **kw_args):
        all_indices = list()
        all_values = list()

        for group_name, dof_values in kw_args.items():
            if group_name not in self._groups:
                raise Exception('There is no group named %s.' % group_name)

            group_indices = self._groups[group_name]
            if len(group_indices) != len(dof_values):
                raise Exception('Expected %d values; got %d.'\
                                % (len(group_indices), len(dof_values)))

            all_indices.extend(group_indices)
            all_values.extend(dof_values)

        self._configs[name] = (all_indices, all_values)
        return all_indices, all_values

    def get_configuration(self, name):
        if name in self._configs:
            return self._configs[name]
        else:
            raise Exception('There is no configuration named %s.' % name)
