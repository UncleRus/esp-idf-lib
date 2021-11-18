# frozen_string_literal: true

require_relative "component"

# A class that represents metadaata, `.eil.yml`
class Metadata
  # path: path to component root directory
  def initialize(path)
    raise ArgumentError, "path is missing" unless path

    @path = path
    @name = File.basename(path)
    raise ArgumentError, "path `#{path}` does not have basename" if @name.empty?

    metadata
  end

  attr_reader :path, :name

  def metadata
    return @metadata if @metadata

    file = File.join(path, ".eil.yml")
    @metadata = YAML.safe_load(File.read(file))
  rescue StandardError => e
    warn "failed to open `#{file}`. does component `#{File.basename(path)}` have `.eil.yml` file?"
    raise e
  end

  def components?
    metadata.key?("components")
  end

  def components
    metadata["components"].map { |c| Component.new(c) }
  end

  def to_s
    name
  end
end
