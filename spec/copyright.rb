# frozen_string_literal: true

require_relative "person"

class Copyright
  def initialize(hash)
    @metadata = hash
    raise ArgumentError, "missing argument `name`" unless hash.key?("name")
    raise ArgumentError, "missing argument `year`" unless hash.key?("year")
  end

  def name
    metadata["name"]
  end

  def year
    metadata["year"]
  end

  def author
    Person.new(name)
  end
end
