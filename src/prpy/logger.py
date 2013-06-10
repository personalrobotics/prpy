import collections, logging, sys

class ColoredFormatter(logging.Formatter):
    def __init__(self, default):
        self._default_formatter = default
        self._color_table = collections.defaultdict(lambda: list())
        self._color_table[logging.CRITICAL] = [ 'red' ]
        self._color_table[logging.ERROR] = [ 'red' ]
        self._color_table[logging.WARNING] = [ 'yellow' ]
        self._color_table[logging.DEBUG] = [ 'green' ]

        # Import termcolor now to fail-fast.
        import termcolor
        self.termcolor = termcolor

    def format(self, record):
        color_options = self._color_table[record.levelno]
        message = self._default_formatter.format(record)
        return self.termcolor.colored(message, *color_options)

def initialize_logging():
    formatter = logging.Formatter('[%(levelname)s] [%(name)s:%(filename)s:%(lineno)d]:%(funcName)s: %(message)s')

    # Remove all of the existing handlers.
    base_logger = logging.getLogger()
    for handler in base_logger.handlers:
        print 'REMOVING', handler
        base_logger.removeHandler(handler)

    # Add the custom handler.
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(formatter)
    base_logger.addHandler(handler)
    base_logger.setLevel(logging.INFO)

    # Colorize logging output if the termcolor package is available.
    try:
        color_formatter = ColoredFormatter(formatter)
        handler.setFormatter(color_formatter)
    except ImportError:
        logging.warning('Install termcolor to colorize log messages.')

    return base_logger
