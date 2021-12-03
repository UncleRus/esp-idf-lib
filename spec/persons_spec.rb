# frozen_string_literal: true

require_relative "spec_helper"
require_relative "person_list"
require_relative "person"

file = File.join(File.dirname(__FILE__), "..", "persons.yml")

RSpec.describe "Person list metadata #{file}" do
  persons = PersonList.new(file)

  persons.all.each do |person|
    describe "Person #{person}" do
      subject { Person.new(person) }

      it "has a name" do
        expect(subject.name?).to be true
      end

      it "has non-empty name" do
        expect(subject.name).not_to be_empty
      end

      it "has one or more of contact information" do
        has_contact = subject.email? || subject.gh_id? || subject.full_name? || subject.website?
        expect(has_contact).to be true
      end

      it "is unique in persons.yml" do
        expect(persons.lookup(subject.name).length).to be 1
      end
    end
  end
end
