import os
import typing as t

__all__ = [
    'MetadataError',
    'MetadataNotFoundError',
    'StructureError',
    'InvalidFieldsError',
    'InvalidNameError',
    'InvalidItemError',
    'InvalidGroupError',
    'InvalidPersonError',
    'InvalidLicenseError',
    'InvalidTargetError',
    'InvalidThreadSafetyError',
]


class MetadataError(RuntimeError):
    pass


class MetadataNotFoundError(MetadataError):
    def __init__(self, path: os.PathLike):
        super().__init__('Metadata not found: "%s"' % path)


class StructureError(MetadataError):
    def __init__(self, ctx: str):
        super().__init__('[%s]: Bad metadata structure' % ctx)


class InvalidFieldsError(MetadataError):
    def __init__(self, ctx: str, expected: t.Set[str], got: t.Set[str]):
        super().__init__('[%s]: Bad metadata. Expected fields: %r, got %r' % (ctx, expected, got))


class InvalidNameError(MetadataError):
    def __init__(self, dirname: str, metaname: str):
        super().__init__('[%s]: Invalid component name: "%s", must be "%s"' % (dirname, metaname, dirname))


class InvalidItemError(MetadataError):
    def __init__(self, ctx: str, item_name: str, item_value: str):
        super().__init__('[%s]: Invalid %s: "%s"' % (ctx, item_name, item_value))


class InvalidGroupError(InvalidItemError):
    def __init__(self, ctx: str, value: str):
        super().__init__(ctx, 'group', value)


class InvalidPersonError(InvalidItemError):
    def __init__(self, ctx: str, value: str):
        super().__init__(ctx, 'person', value)


class InvalidLicenseError(InvalidItemError):
    def __init__(self, ctx: str, value: str):
        super().__init__(ctx, 'license', value)


class InvalidTargetError(InvalidItemError):
    def __init__(self, ctx: str, value: str):
        super().__init__(ctx, 'target', value)


class InvalidThreadSafetyError(InvalidItemError):
    def __init__(self, ctx: str, value: str):
        super().__init__(ctx, 'thread_safe', value)
