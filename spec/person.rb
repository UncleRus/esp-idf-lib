# frozen_string_literal: true

require "yaml"

# A class that repesents Peson
class Person
  PERSON_FILE = File.expand_path(File.join(File.dirname("__FILE__"), "persons.yml")).freeze

  def initialize(arg)
    validate_arg(arg)
    validate_keys(arg) if arg.is_a? Hash

    name = if arg.is_a? String
             arg
           else
             arg["name"]
           end
    @metadata = lookup_person(name)
  end

  attr_reader :metadata, :persons

  def validate_keys(hash)
    raise ArgumentError, "missing key: `name`" unless hash.key?("name")
  end

  def valid_arg_class?(arg)
    arg.is_a?(String) || arg.is_a?(Hash)
  end

  def validate_arg(arg)
    raise ArgumentError, "String or dict is ecpected, but got `#{arg.class}`" unless valid_arg_class?(arg)
  end

  def load_person_file
    File.read(PERSON_FILE)
  end

  def parse(string)
    return @persons if @persons

    @persons = YAML.safe_load(string)
  rescue StandardError => e
    warn "failed to parse #{PERSON_FILE} as YAML"
    raise e
  end

  def lookup_person(name)
    parse(load_person_file)
    person = persons.select { |p| p["name"] == name }
    raise ArgumentError, "cannot find Person with name `#{name}` in #{PERSON_FILE}" unless person
    raise ArgumentError, "Person with name `#{name}` has duplicated entry in #{PERSON_FILE}" if person.length > 1

    person.first
  end

  def name?
    metadata.key?("name")
  end

  def name
    metadata["name"]
  end

  def full_name?
    metadata.key?("full_name")
  end

  def full_name
    metadata["full_name"]
  end

  def gh_id?
    metadata.key?("gh_id")
  end

  def gh_id
    metadata["gh_id"]
  end

  def email?
    metadata.key?("email")
  end

  def email
    metadata["email"]
  end

  def website?
    metadata.key?("website")
  end

  def website
    metadata["website"]
  end

  def to_s
    metadata["name"]
  end
end
